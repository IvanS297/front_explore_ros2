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
        self.declare_parameter('debug', False)
        self.is_in_debug_mode = bool(self.get_parameter('debug').value)

        # ── Параметры — читаются из params.yaml ─────────────
        self.declare_parameter('num_explore_fails_before_finish', 60)
        self.declare_parameter('min_map_cells', 300)
        self.declare_parameter('min_frontier_size', 3)
        self.declare_parameter('max_frontiers_to_check', 8)
        self.declare_parameter('a_star_cost_weight', 10.0)
        self.declare_parameter('frontier_size_cost_weight', 1.0)
        self.declare_parameter('exploration_rate', 2.0)

        self.NUM_EXPLORE_FAILS_BEFORE_FINISH = self.get_parameter('num_explore_fails_before_finish').value
        self.MIN_MAP_CELLS                   = self.get_parameter('min_map_cells').value
        self.MIN_FRONTIER_SIZE               = self.get_parameter('min_frontier_size').value
        self.MAX_FRONTIERS_TO_CHECK          = self.get_parameter('max_frontiers_to_check').value
        self.A_STAR_COST_WEIGHT              = self.get_parameter('a_star_cost_weight').value
        self.FRONTIER_SIZE_COST_WEIGHT       = self.get_parameter('frontier_size_cost_weight').value
        self.EXPLORATION_RATE                = self.get_parameter('exploration_rate').value

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

        self.no_path_found_counter = 0
        self.no_frontiers_found_counter = 0
        self.is_finished_exploring = False
        self._tf_ready = False

    def update_odometry(self, msg: "Union[Odometry, None]" = None):
        """
        Updates the current pose of the robot.
        """
        try:
            trans = self.tf_buffer.lookup_transform(
                "map", "base_footprint",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            # Логируем редко — не засоряем терминал
            self.get_logger().warn(
                f"TF error: {ex}", throttle_duration_sec=3.0
            )
            self._tf_ready = False
            return
        translation = trans.transform.translation
        rotation = trans.transform.rotation
        self._tf_ready = True
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

        A_STAR_COST_WEIGHT = self.A_STAR_COST_WEIGHT
        FRONTIER_SIZE_COST_WEIGHT = self.FRONTIER_SIZE_COST_WEIGHT

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
        MAX_NUM_FRONTIERS_TO_CHECK = self.MAX_FRONTIERS_TO_CHECK
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
        rate = self.create_rate(self.EXPLORATION_RATE)
        while rclpy.ok():

            # ── 1. Ждём TF ────────────────────────────────────────────────
            if not self._tf_ready or self.pose is None:
                self.get_logger().info(
                    "Жду TF map→base_footprint...",
                    throttle_duration_sec=3.0
                )
                rate.sleep()
                continue

            # ── 2. Ждём карту ─────────────────────────────────────────────
            if self.map is None:
                self.get_logger().info(
                    "Жду карту...", throttle_duration_sec=3.0
                )
                rate.sleep()
                continue

            map_cells = self.map.info.width * self.map.info.height
            if map_cells < self.MIN_MAP_CELLS:
                self.get_logger().info(
                    f"Карта {self.map.info.width}x{self.map.info.height}"
                    f" ({map_cells} кл.) — слишком маленькая, жду {self.MIN_MAP_CELLS}+",
                    throttle_duration_sec=3.0
                )
                rate.sleep()
                continue

            # ── 3. Проверяем что стартовая клетка внутри карты ───────────
            start = PathPlanner.world_to_grid(self.map, self.pose.position)
            if not PathPlanner.is_cell_in_bounds(self.map, start):
                self.get_logger().warn(
                    f"Робот вне карты: grid={start}, "
                    f"карта {self.map.info.width}x{self.map.info.height}",
                    throttle_duration_sec=3.0
                )
                rate.sleep()
                continue

            # ── 4. BFS поиск фронтиров ────────────────────────────────────
            frontier_list, frontier_cells = FrontierSearch.search(
                self.map, start, self.is_in_debug_mode, self.MIN_FRONTIER_SIZE
            )

            if frontier_list is None:
                rate.sleep()
                continue

            self.get_logger().info(
                f"Фронтиров: {len(frontier_list.frontiers)} | "
                f"Карта: {self.map.info.width}x{self.map.info.height} | "
                f"Старт: {start} | "
                f"Фейлов: {self.no_frontiers_found_counter}/{self.NUM_EXPLORE_FAILS_BEFORE_FINISH}",
                throttle_duration_sec=2.0
            )

            if self.is_in_debug_mode and frontier_cells:
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