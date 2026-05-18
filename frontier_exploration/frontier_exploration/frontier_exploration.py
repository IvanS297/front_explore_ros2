#!/usr/bin/env python3

import os
import threading
import subprocess
import numpy as np
from typing import Union
from .path_planner import PathPlanner
from .frontier_search import FrontierSearch
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from frontier_msgs.msg import FrontierList
from tf_transformations import euler_from_quaternion
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from ament_index_python.packages import get_package_share_directory

class FrontierExploration(Node):
    def __init__(self):
        super().__init__('frontier_exploration')
        """
        Class constructor
        """

        # Set if in debug mode
        self.declare_parameter('debug', True)
        self.is_in_debug_mode = self.get_parameter('debug').value == 'true'

        # Publishers
        self.pure_pursuit_pub = self.create_publisher(Path, '/pure_pursuit/path', 10)

        if self.is_in_debug_mode:
            self.frontier_cells_pub = self.create_publisher(GridCells, '/frontier_exploration/frontier_cells', 10)
            self.start_pub = self.create_publisher(GridCells, '/frontier_exploration/start', 10)
            self.goal_pub = self.create_publisher(GridCells, '/frontier_exploration/goal', 10)
            self.cspace_pub = self.create_publisher(GridCells, '/cspace', 10)
            self.cost_map_pub = self.create_publisher(OccupancyGrid, '/cost_map', 10)

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.update_odometry, 10)
        self.create_subscription(OccupancyGrid, '/map', self.update_map, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lock = threading.Lock()
        self.pose = None
        self.map = None

        self.NUM_EXPLORE_FAILS_BEFORE_FINISH = 30
        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False

    def update_odometry(self, msg: "Union[Odometry, None]" = None):
        """
        Updates the current pose of the robot.
        """
        try:
            when = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("map", "base_footprint", when)
        except TransformException as ex:
            self.get_logger().warn(f"TF error: {ex}")
            return
        translation = trans.transform.translation
        rotation = trans.transform.rotation

        self.pose = Pose(
            position=Point(x=translation.x, y=translation.y),
            orientation=rotation
        )

    def update_map(self, msg: OccupancyGrid):
        """
        Updates the current map.
        This method is a callback bound to a Subscriber.
        :param msg [OccupancyGrid] The current map information.
        """
        self.map = msg

    def save_map(self):
        # Get the path of the current package
        package_share = get_package_share_directory('frontier_exploration')

        # Construct the path to the map
        map_path = os.path.join(package_share, 'map', 'map')
        if not os.path.exists(os.path.dirname(map_path)):
            os.makedirs(os.path.dirname(map_path))

        # Run map_saver
        subprocess.call(["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path])

        self.update_odometry()

        if self.pose is None:
            self.get_logger().error("Failed to get pose")
            return

        # Save the robot's position and orientation
        position = self.pose.position
        orientation = self.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        )
        with open(os.path.join(package_share, "map/pose.txt"), "w") as f:
            f.write(f"{position.x} {position.y} {position.z} {yaw} {pitch} {roll}\n")

    @staticmethod
    def get_top_frontiers(frontiers, n):
        # Sort the frontiers by size in descending order
        sorted_frontiers = sorted(
            frontiers, key=lambda frontier: frontier.size, reverse=True
        )

        # Return the top n frontiers
        return sorted_frontiers[:n]

    def publish_cost_map(self, mapdata: OccupancyGrid, cost_map: np.ndarray):
        # Create an OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "map"
        grid.info.resolution = mapdata.info.resolution
        grid.info.width = cost_map.shape[1]
        grid.info.height = cost_map.shape[0]
        grid.info.origin = mapdata.info.origin

        # Normalize the cost map to the range [0, 100] and convert it to integers
        cost_map_normalized = (cost_map / np.max(cost_map) * 100).astype(np.int8)

        # Flatten the cost map and convert it to a list
        grid.data = cost_map_normalized.flatten().tolist()

        # Publish the OccupancyGrid message
        self.cost_map_pub.publish(grid)

    def check_if_finished_exploring(self):
        # Publish empty path to stop the robot
        self.pure_pursuit_pub.publish(Path())

        # If no frontiers or paths are found for a certain number of times, finish exploring
        if (
            self.no_frontiers_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
            or self.no_path_found_counter >= self.NUM_EXPLORE_FAILS_BEFORE_FINISH
        ):
            self.get_logger().info("Done exploring!")
            self.save_map()
            self.get_logger().info("Saved map")
            self.is_finished_exploring = True

    def explore_frontier(self, frontier_list: FrontierList):
        # If finished exploring, no pose, no map, or no frontier list, return
        if self.is_finished_exploring or self.pose is None or self.map is None:
            return

        frontiers = frontier_list.frontiers

        # If no frontiers are found, check if finished exploring
        if not frontiers:
            self.get_logger().info("No frontiers")
            self.no_frontiers_found_counter += 1
            self.check_if_finished_exploring()
            return
        else:
            self.no_frontiers_found_counter = 0

        A_STAR_COST_WEIGHT = 10.0
        FRONTIER_SIZE_COST_WEIGHT = 1.0

        # Calculate the C-space
        cspace, cspace_cells = PathPlanner.calc_cspace(self.map, self.is_in_debug_mode)
        # if cspace_cells is not None:
        #     self.cspace_pub.publish(cspace_cells)

        # Calculate the cost map
        cost_map = PathPlanner.calc_cost_map(self.map)
        if self.is_in_debug_mode:
            self.publish_cost_map(self.map, cost_map)

        # Get the start
        start = PathPlanner.world_to_grid(self.map, self.pose.position)

        # Execute A* for every frontier
        lowest_cost = float("inf")
        best_path = None

        # Check only the top frontiers in terms of size
        MAX_NUM_FRONTIERS_TO_CHECK = 8
        top_frontiers = FrontierExploration.get_top_frontiers(
            frontiers, MAX_NUM_FRONTIERS_TO_CHECK
        )

        starts = []
        goals = []

        # Log how many frontiers are being explored
        self.get_logger().info(f"Exploring {len(top_frontiers)} frontiers")

        for frontier in top_frontiers:
            # Get goal
            goal = PathPlanner.world_to_grid(self.map, frontier.centroid)

            # Execute A*
            path, a_star_cost, start, goal = PathPlanner.a_star(
                cspace, cost_map, start, goal
            )

            # If in debug mode, append start and goal
            if self.is_in_debug_mode:
                starts.append(start)
                goals.append(goal)

            if path is None or a_star_cost is None:
                continue

            # Calculate cost
            cost = (A_STAR_COST_WEIGHT * a_star_cost) + (
                FRONTIER_SIZE_COST_WEIGHT / frontier.size
            )

            # Update best path
            if cost < lowest_cost:
                lowest_cost = cost
                best_path = path

        # If in debug mode, publish the start and goal
        if self.is_in_debug_mode:
            self.start_pub.publish(PathPlanner.get_grid_cells(self.map, starts))
            self.goal_pub.publish(PathPlanner.get_grid_cells(self.map, goals))

        # If a path was found, publish it
        if best_path:
            self.get_logger().info(f"Found best path with cost {lowest_cost}")
            start = best_path[0]
            path = PathPlanner.path_to_message(self.map, best_path)
            self.pure_pursuit_pub.publish(path)
            self.no_path_found_counter = 0
        # If no path was found, check if finished exploring
        else:
            self.get_logger().info("No paths found")
            self.no_path_found_counter += 1
            self.check_if_finished_exploring()

    def run(self):
        rate = self.create_rate(20) # Hz
        while rclpy.ok():
            if self.pose is None or self.map is None:
                continue

            # Get the start position of the robot
            start = PathPlanner.world_to_grid(self.map, self.pose.position)

            # Get frontiers
            frontier_list, frontier_cells = FrontierSearch.search(
                self.map, start, self.is_in_debug_mode
            )

            if frontier_list is None:
                continue

            # Publish frontier cells if in debug mode
            if self.is_in_debug_mode:
                self.frontier_cells_pub.publish(
                    PathPlanner.get_grid_cells(self.map, frontier_cells)
                )

            self.explore_frontier(frontier_list)

            rate.sleep()

def main(args=None):
    rclpy.init(args=args)
    node = FrontierExploration()
    thread = threading.Thread(target=node.run, daemon=True)
    thread.start()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()