import json
from ast import literal_eval
from functools import partial

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

PATH_BLOCKED_OCCUPANCY_MIN = 51
PREFLIGHT_TIMEOUT_SEC = 10.0


class ScoreAndPostNode(Node):
    def __init__(self):
        super().__init__('score_and_post')
        self.get_logger().debug('score_and_post node has started')

        # State Flags
        self.exploration_active = True
        self.startup_ready = False  # Wait for SLAM to build initial map
        self._startup_timer = self.create_timer(20.0, self._startup_ready_callback)
        self.get_logger().info('Waiting 20s for SLAM to build initial map...')

        # Service
        self.toggle_service = self.create_service(
            SetBool,
            'toggle_exploration',
            self.toggle_callback
        )

        self.status_pub = self.create_publisher(String, '/mission_status', 10)

        self.bfs_data = {}
        self.frontiers = []
        self.robot_x = None
        self.robot_y = None
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 1.0
        self.map_width = 0
        self.map_height = 0
        self.navigation_in_progress = False
        self.preflight_in_progress = False
        self.current_preflight_goal_handle = None
        self.current_goal_handle = None
        self.active_goal_frontier = None
        self.last_goal_frontier = None
        self.current_goal_id = 0
        self.canceled_preflight_goal_ids = set()
        self.preflight_timeout_timer = None
        self.scored_frontiers = {}
        self.tf_buffer = tf2_ros.Buffer(
            cache_time=rclpy.duration.Duration(seconds=5.0)
        )
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.frontiers_subscription()
        self.bfs_subscription()
        self.map_data_subscription()
        self.nav2_server_subscription()
        self.nav2_goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.blacklist = {}

        # Timeout settings
        self.declare_parameter('nav2_goal_timeout_sec', 60.0)
        self.goal_timeout_sec = self.get_parameter('nav2_goal_timeout_sec').get_parameter_value().double_value
        
        self.active_goal_start_time = None
        
        self.timeout_timer = self.create_timer(1.0, self.timeout_monitor_callback)

    def frontiers_subscription(self):
        self.frontiers_subscriber = self.create_subscription(
            String,
            'frontiers',
            self.frontiers_callback,
            10,
        )

    def bfs_subscription(self):
        self.bfs_subscriber = self.create_subscription(
            String,
            'bfs_distance_transform',
            self.bfs_callback,
            10,
        )

    def map_data_subscription(self):
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10,
        )

    def nav2_server_subscription(self):
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose',
        )
        self.nav2_connected = False
        self.navigation_in_progress = False
        # Don't block — check server readiness in a timer so the node can spin
        self.nav2_check_timer = self.create_timer(1.0, self._check_nav2_servers)

    def _check_nav2_servers(self):
        if self.nav_client.server_is_ready() and self.compute_path_client.server_is_ready():
            self.nav2_connected = True
            self.nav2_check_timer.cancel()
            self.destroy_timer(self.nav2_check_timer)
            self.get_logger().debug('Connected to navigation action servers')
        else:
            self.get_logger().debug('Waiting for Nav2 action servers...')

    def _startup_ready_callback(self):
        self.startup_ready = True
        self.get_logger().info('Startup delay complete — exploration can begin.')
        # Cancel timer so it doesn't fire repeatedly
        if hasattr(self, '_startup_timer') and self._startup_timer:
            self._startup_timer.cancel()

    def toggle_callback(self, request, response):
        self.exploration_active = request.data
        
        if self.exploration_active:
            self.get_logger().info('Exploration RESUMED by coordinator.')
            response.message = 'Exploration is now ACTIVE.'
        else:
            self.get_logger().info('Exploration PAUSED by coordinator.')
            response.message = 'Exploration is now PAUSED.'
            
            # The Kill Switch
            if self.navigation_in_progress and self.current_goal_handle:
                self.get_logger().debug('Canceling active Nav2 goal...')
                self.current_goal_handle.cancel_goal_async()
            if self.preflight_in_progress and self.current_preflight_goal_handle:
                self.get_logger().debug('Canceling active preflight goal...')
                self.current_preflight_goal_handle.cancel_goal_async()

            self.clear_navigation_state()

        response.success = True
        return response

    def map_callback(self, msg):
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution

        self.map_formatted_data = {}
        self.occupancy_grid = msg.data
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin = msg.info.origin
        for i in range(self.map_height):
            for j in range(self.map_width):
                index = i * self.map_width + j
                self.map_formatted_data[(j, i)] = self.occupancy_grid[index]

        self.maybe_cancel_blocked_preflight()

    def update_robot_position(self):
        if self.map_width <= 0 or self.map_height <= 0 or self.map_resolution <= 0.0:
            self.robot_x = None
            self.robot_y = None
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.robot_x = None
            self.robot_y = None
            return None

        translation = transform.transform.translation
        map_x, map_y = self.world_to_map(translation.x, translation.y)
        if 0 <= map_x < self.map_width and 0 <= map_y < self.map_height:
            self.robot_x = map_x
            self.robot_y = map_y
            return (map_x, map_y)

        self.robot_x = None
        self.robot_y = None
        return None

    def frontier_to_world(self, frontier):
        frontier_x, frontier_y = frontier
        world_x = self.map_origin_x + frontier_x * self.map_resolution
        world_y = self.map_origin_y + frontier_y * self.map_resolution
        return world_x, world_y

    def build_goal_pose(self, frontier):
        world_x, world_y = self.frontier_to_world(frontier)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = world_x
        goal.pose.position.y = world_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def frontier_key(self, frontier):
        return str((int(frontier[0]), int(frontier[1])))

    def clear_navigation_state(self):
        self.stop_preflight_timeout()
        self.navigation_in_progress = False
        self.preflight_in_progress = False
        self.current_preflight_goal_handle = None
        self.current_goal_handle = None
        self.active_goal_frontier = None
        self.active_goal_start_time = None

    def timeout_monitor_callback(self):
        now = self.get_clock().now()
            
        if self.navigation_in_progress and self.active_goal_start_time is not None:
            goal_duration = (now - self.active_goal_start_time).nanoseconds / 1e9
            if goal_duration >= self.goal_timeout_sec:
                self.get_logger().warning(f'Nav2 goal timed out after {goal_duration:.1f}s. Blacklisting and retrying.')
                
                if self.active_goal_frontier:
                    self.blacklist_frontier_cluster(self.active_goal_frontier)
                
                if self.current_goal_handle:
                    self.current_goal_handle.cancel_goal_async()
                    
                self.clear_navigation_state()
                self.filter_frontiers()

    def stop_preflight_timeout(self):
        if self.preflight_timeout_timer is None:
            return

        self.preflight_timeout_timer.cancel()
        self.destroy_timer(self.preflight_timeout_timer)
        self.preflight_timeout_timer = None

    def start_preflight_timeout(self, goal_id, goal_frontier):
        self.stop_preflight_timeout()
        self.preflight_timeout_timer = self.create_timer(
            PREFLIGHT_TIMEOUT_SEC,
            partial(
                self.preflight_timeout_callback,
                goal_id=goal_id,
                goal_frontier=goal_frontier,
            ),
        )

    def preflight_timeout_callback(self, goal_id, goal_frontier):
        self.stop_preflight_timeout()
        if not self.is_active_preflight(goal_id, goal_frontier):
            return

        self.get_logger().warning(
            'ComputePathToPose timed out for frontier '
            f'{goal_frontier} after {PREFLIGHT_TIMEOUT_SEC:.1f}s'
        )
        self.abandon_preflight_and_retry(goal_id, goal_frontier)

    def abandon_preflight_and_retry(self, goal_id, goal_frontier):
        goal_handle = None
        if self.is_active_preflight(goal_id, goal_frontier):
            goal_handle = self.current_preflight_goal_handle

        self.clear_navigation_state()

        if goal_handle is None:
            self.canceled_preflight_goal_ids.add(goal_id)
            self.get_logger().warning(
                'ComputePathToPose goal handle was not available yet for '
                f'frontier {goal_frontier}; will cancel if it arrives later'
            )
        else:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(
                partial(
                    self.compute_path_cancel_callback,
                    goal_frontier=goal_frontier,
                )
            )

        self.filter_frontiers()

    def compute_path_cancel_callback(self, future, goal_frontier):
        try:
            cancel_response = future.result()
        except Exception as error:
            self.get_logger().error(
                'Failed to cancel ComputePathToPose goal for frontier '
                f'{goal_frontier}: {error}'
            )
            return

        if cancel_response.goals_canceling:
            self.get_logger().debug(
                'Canceled ComputePathToPose goal for frontier '
                f'{goal_frontier}'
            )
            return

        self.get_logger().warning(
            'ComputePathToPose cancellation request returned no active goal '
            f'for frontier {goal_frontier}'
        )

    def get_active_preflight_block_reason(self):
        if not self.preflight_in_progress or self.active_goal_frontier is None:
            return None

        frontier = self.active_goal_frontier
        frontier_key = self.frontier_key(frontier)

        if frontier_key in self.blacklist:
            return 'it has already been blacklisted'

        if hasattr(self, 'map_formatted_data'):
            occupancy = self.map_formatted_data.get(frontier)
            if (
                occupancy is not None
                and PATH_BLOCKED_OCCUPANCY_MIN <= occupancy <= 100
            ):
                return f'its map cell is occupied ({occupancy})'

        if isinstance(self.frontiers, list):
            latest_frontier_keys = {
                self.frontier_key(candidate)
                for candidate in self.frontiers
                if isinstance(candidate, (list, tuple)) and len(candidate) == 2
            }
            if frontier_key not in latest_frontier_keys:
                return 'it is no longer present in the latest frontier list'

        if isinstance(self.bfs_data, dict) and self.bfs_data:
            bfs_score = self.bfs_data.get(frontier_key)
            if bfs_score == float('inf'):
                return 'its BFS score is now infinite'

        return None

    def maybe_cancel_blocked_preflight(self):
        reason = self.get_active_preflight_block_reason()
        if reason is None:
            return False

        goal_id = self.current_goal_id
        goal_frontier = self.active_goal_frontier
        self.get_logger().warning(
            'Active preflight frontier '
            f'{goal_frontier} became blocked because {reason}; '
            'canceling ComputePathToPose and moving on'
        )
        self.abandon_preflight_and_retry(goal_id, goal_frontier)
        return True

    def blacklist_frontier_cluster(self, frontier):
        if frontier is None:
            return

        self.blacklist[self.frontier_key(frontier)] = (
            -float('inf'),
            float('inf'),
            -float('inf'),
        )
        self.get_logger().debug(
            f'Added {frontier} to blacklist with score '
            f'{self.blacklist[self.frontier_key(frontier)]}'
        )

    def bfs_callback(self, msg):
        self.get_logger().debug('Received BFS distance transform data')
        try:
            self.bfs_data = json.loads(msg.data)
        except json.JSONDecodeError as error:
            self.get_logger().error(f'Failed to decode BFS data: {error}')
            return

        if self.maybe_cancel_blocked_preflight():
            return
        self.filter_frontiers()

    def frontiers_callback(self, msg):
        self.get_logger().debug('Received frontiers data')
        try:
            self.frontiers = json.loads(msg.data)
            if self.maybe_cancel_blocked_preflight():
                return
            self.filter_frontiers()
        except json.JSONDecodeError as error:
            self.get_logger().error(f'Failed to decode frontiers data: {error}')

    def is_active_preflight(self, goal_id, goal_frontier):
        return (
            goal_id == self.current_goal_id
            and self.preflight_in_progress
            and self.active_goal_frontier == goal_frontier
        )

    def is_active_navigation(self, goal_id, goal_frontier):
        return (
            goal_id == self.current_goal_id
            and self.navigation_in_progress
            and self.active_goal_frontier == goal_frontier
        )

    def start_frontier_preflight(self, best_frontier):
        if not self.nav2_connected:
            return
        if self.navigation_in_progress or self.preflight_in_progress:
            self.get_logger().debug(
                'Another frontier attempt is already in progress'
            )
            return

        frontier_coords = literal_eval(best_frontier)
        frontier = (int(frontier_coords[0]), int(frontier_coords[1]))
        goal_pose = self.build_goal_pose(frontier)

        self.preflight_in_progress = True
        self.active_goal_frontier = frontier
        self.last_goal_frontier = frontier
        self.current_goal_id += 1
        goal_id = self.current_goal_id

        compute_goal = ComputePathToPose.Goal()
        compute_goal.goal = goal_pose
        compute_goal.planner_id = ''
        compute_goal.use_start = False

        self.get_logger().debug(
            f'Preflighting frontier {frontier} by asking Nav2 to compute '
            'a path before sending NavigateToPose'
        )
        self.current_preflight_goal_handle = None
        future = self.compute_path_client.send_goal_async(compute_goal)
        self.start_preflight_timeout(goal_id, frontier)
        future.add_done_callback(
            partial(
                self.compute_path_goal_response_callback,
                goal_id=goal_id,
                goal_frontier=frontier,
                goal_pose=goal_pose,
            )
        )

    def compute_path_goal_response_callback(
        self,
        future,
        goal_id,
        goal_frontier,
        goal_pose,
    ):
        try:
            goal_handle = future.result()
        except Exception as error:
            if goal_id in self.canceled_preflight_goal_ids:
                self.canceled_preflight_goal_ids.discard(goal_id)
                self.get_logger().warning(
                    'ComputePathToPose send completed after frontier '
                    f'{goal_frontier} was abandoned: {error}'
                )
                return

            self.clear_navigation_state()
            self.get_logger().error(
                'Failed to send ComputePathToPose goal for frontier '
                f'{goal_frontier}: {error}'
            )
            self.filter_frontiers()
            return

        if goal_id in self.canceled_preflight_goal_ids:
            self.canceled_preflight_goal_ids.discard(goal_id)
            if goal_handle.accepted:
                self.get_logger().warning(
                    'ComputePathToPose response arrived after frontier '
                    f'{goal_frontier} was abandoned; canceling it now'
                )
                cancel_future = goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    partial(
                        self.compute_path_cancel_callback,
                        goal_frontier=goal_frontier,
                    )
                )
            else:
                self.get_logger().warning(
                    'ComputePathToPose response arrived after frontier '
                    f'{goal_frontier} was abandoned and the goal was '
                    'already rejected'
                )
            return

        if not self.is_active_preflight(goal_id, goal_frontier):
            self.get_logger().debug(
                f'Ignoring stale ComputePathToPose response for goal id {goal_id}'
            )
            return

        try:
            if not goal_handle.accepted:
                self.clear_navigation_state()
                self.get_logger().warning(
                    'ComputePathToPose goal was rejected for frontier '
                    f'{goal_frontier}'
                )
                self.filter_frontiers()
                return
        except Exception as error:
            self.clear_navigation_state()
            self.get_logger().error(
                'Error while handling ComputePathToPose goal response for '
                f'frontier {goal_frontier}: {error}'
            )
            self.filter_frontiers()
            return

        self.current_preflight_goal_handle = goal_handle
        self.get_logger().debug(
            f'ComputePathToPose goal accepted for frontier {goal_frontier}'
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            partial(
                self.compute_path_result_callback,
                goal_id=goal_id,
                goal_frontier=goal_frontier,
                goal_pose=goal_pose,
            )
        )

    def compute_path_result_callback(
        self,
        future,
        goal_id,
        goal_frontier,
        goal_pose,
    ):
        if not self.is_active_preflight(goal_id, goal_frontier):
            self.get_logger().debug(
                f'Ignoring stale computed path result for goal id {goal_id}'
            )
            return

        self.stop_preflight_timeout()
        try:
            result = future.result()
        except Exception as error:
            self.clear_navigation_state()
            self.get_logger().error(
                f'Failed to compute path for frontier {goal_frontier}: {error}'
            )
            self.filter_frontiers()
            return

        path = result.result.path
        if not path.poses:
            self.blacklist_frontier_cluster(goal_frontier)
            self.clear_navigation_state()
            self.get_logger().debug(
                'Nav2 could not produce a usable path for frontier cluster '
                f'{goal_frontier}; blacklisting it'
            )
            self.filter_frontiers()
            return

        path_is_clear, blocked_cells = self.is_path_clear(path)
        if not path_is_clear:
            self.blacklist_frontier_cluster(goal_frontier)
            self.clear_navigation_state()
            self.get_logger().debug(
                'Manual path validation found blocked cells for frontier '
                f'cluster {goal_frontier}; blocked cells sample: '
                f'{blocked_cells[:10]}'
            )
            self.filter_frontiers()
            return

        self.get_logger().debug(
            f'Computed path with {len(path.poses)} poses for frontier '
            f'{goal_frontier}; manual path validation passed'
        )

        navigate_goal = NavigateToPose.Goal()
        navigate_goal.pose = goal_pose

        self.current_preflight_goal_handle = None
        self.preflight_in_progress = False
        self.navigation_in_progress = True
        self.active_goal_start_time = self.get_clock().now()
        self.nav2_goal_pub.publish(goal_pose)
        future = self.nav_client.send_goal_async(navigate_goal)
        future.add_done_callback(
            partial(
                self.goal_response_callback,
                goal_id=goal_id,
                goal_frontier=goal_frontier,
            )
        )
        world_x, world_y = self.frontier_to_world(goal_frontier)
        self.get_logger().debug(
            'Path validated, sending NavigateToPose goal to frontier at '
            f'world coordinates ({world_x:.2f}, {world_y:.2f})'
        )

    def world_to_map(self, world_x, world_y):
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def is_path_clear(self, path):
        if not hasattr(self, 'map_formatted_data'):
            self.get_logger().warning(
                'Map data not yet available; treating path as blocked.'
            )
            return False, []

        blocked_cells = []
        for pose in path.poses:
            map_cell = self.world_to_map(
                pose.pose.position.x,
                pose.pose.position.y,
            )
            occupancy = self.map_formatted_data.get(map_cell)
            if (
                occupancy is None
                or occupancy < 0
                or PATH_BLOCKED_OCCUPANCY_MIN <= occupancy <= 100
            ):
                blocked_cells.append(map_cell)

        return not blocked_cells, blocked_cells

    def calculate_distance_score(self, frontier, robot_position):
        if robot_position is None:
            return 0.0

        frontier_x, frontier_y = frontier
        robot_x, robot_y = robot_position
        distance = (
            (frontier_x - robot_x) ** 2 + (frontier_y - robot_y) ** 2
        ) ** 0.5
        return distance

    def filter_frontiers(self):
        if not self.exploration_active:
            return

        if not self.startup_ready:
            return

        if self.navigation_in_progress or self.preflight_in_progress:
            self.get_logger().debug(
                'Frontier attempt already in progress - skipping scoring'
            )
            return

        if not hasattr(self, 'frontiers') or not isinstance(self.frontiers, list) or not self.frontiers:
            # Don't declare complete if map is still tiny — frontiers may appear
            # once the robot moves and SLAM discovers more area.
            map_cells = self.map_width * self.map_height
            if map_cells < 500 or self.exploration_active_duration < 30.0:
                self.get_logger().debug(
                    f'No frontiers yet but map small ({map_cells} cells, '
                    f'{self.exploration_active_duration:.0f}s elapsed). Waiting...')
                return
            if self.exploration_active:
                self.get_logger().info('ZERO valid frontiers remaining. MAP COMPLETE. Terminating exploration.')
                self.exploration_active = False

                # Stop the robot immediately if it is currently moving
                if self.navigation_in_progress and self.current_goal_handle:
                    self.get_logger().debug('Canceling final Nav2 goal...')
                    self.current_goal_handle.cancel_goal_async()
                if self.preflight_in_progress and self.current_preflight_goal_handle:
                    self.get_logger().debug('Canceling active preflight goal...')
                    self.current_preflight_goal_handle.cancel_goal_async()

                self.clear_navigation_state()

                # Tell the coordinator that exploration is done
                msg = {'sender': 'explorer', 'status': 'EXPLORATION_COMPLETE', 'data': None}
                self.status_pub.publish(String(data=json.dumps(msg)))
                self.get_logger().info('Published EXPLORATION_COMPLETE to /mission_status')
            else:
                self.get_logger().warning(
                    'Frontiers data not yet available or invalid or empty; skipping scoring.'
                )
            return

        if not hasattr(self, 'bfs_data') or not isinstance(self.bfs_data, dict):
            self.get_logger().warning(
                'BFS data not yet available; skipping scoring.'
            )
            return

        if not self.bfs_data:
            self.get_logger().debug(
                'BFS data is empty; no distance transform available.'
            )
            return

        robot_position = self.update_robot_position()
        self.scored_frontiers = {}
        for frontier in self.frontiers:
            frontier_key = str(tuple(frontier))
            if frontier_key in self.bfs_data:
                bfs_score = self.bfs_data[frontier_key]
                if bfs_score == float('inf'):
                    self.get_logger().debug(
                        f'Frontier {frontier_key} has infinite BFS score; skipping.'
                    )
                    continue
                distance_score = self.calculate_distance_score(
                    frontier,
                    robot_position,
                )
                blacklist_score = 0
                self.scored_frontiers[frontier_key] = (
                    bfs_score,
                    -distance_score,
                    blacklist_score,
                )
            else:
                self.get_logger().warning(
                    f'Frontier {frontier_key} not found in BFS data; skipping.'
                )

        self.scored_frontiers.update(self.blacklist)
        self.choose_best_frontier()

    def choose_best_frontier(self):
        if not hasattr(self, 'scored_frontiers') or not isinstance(
            self.scored_frontiers,
            dict,
        ):
            self.get_logger().warning(
                'Scored frontiers data not yet available or invalid.'
            )
            return

        if not self.scored_frontiers:
            self.get_logger().debug(
                'No scored frontiers available; cannot select best frontier.'
            )
            return

        if self.navigation_in_progress or self.preflight_in_progress:
            self.get_logger().debug(
                'Frontier attempt already in progress - skipping goal posting'
            )
            return

        best_frontier = None
        ranked_frontiers = sorted(
            self.scored_frontiers,
            key=self.scored_frontiers.get,
            reverse=True,
        )
        
        all_blacklisted = True
        for frontier in ranked_frontiers:
            if frontier not in self.blacklist:
                all_blacklisted = False
                
            if frontier in self.blacklist:
                continue
            if frontier == str(self.last_goal_frontier):
                continue
            best_frontier = frontier
            break

        if all_blacklisted and ranked_frontiers:
            # Don't declare complete too early — map is still growing,
            # new frontiers will appear. Clear blacklist and retry.
            map_cells = self.map_width * self.map_height
            if map_cells < 2000 or self.exploration_active_duration < 60.0:
                self.get_logger().info(
                    f'All frontiers blacklisted but map still small '
                    f'({map_cells} cells, {self.exploration_active_duration:.0f}s). '
                    f'Clearing blacklist to retry.')
                self.blacklist.clear()
                return
            if self.exploration_active:
                self.get_logger().info(
                    'All remaining frontiers are unreachable (blacklisted). MAP COMPLETE. Terminating exploration early.'
                )
                self.exploration_active = False
                self.clear_navigation_state()
                msg = {'sender': 'explorer', 'status': 'EXPLORATION_COMPLETE', 'data': None}
                self.status_pub.publish(String(data=json.dumps(msg)))
            return

        if best_frontier is None:
            self.get_logger().debug(
                'No new scored frontier is available for preflight.'
            )
            return

        self.get_logger().debug(
            f'Best frontier selected: {best_frontier} '
            f'with score {self.scored_frontiers[best_frontier]}'
        )
        try:
            self.start_frontier_preflight(best_frontier)
        except Exception as error:
            self.clear_navigation_state()
            self.get_logger().error(
                f'Failed to start frontier preflight: {error}'
            )

    def goal_response_callback(self, future, goal_id, goal_frontier):
        if not self.is_active_navigation(goal_id, goal_frontier):
            self.get_logger().debug(
                f'Ignoring stale navigation goal response for goal id {goal_id}'
            )
            return

        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.clear_navigation_state()
                self.get_logger().debug(
                    f'Navigation goal rejected for frontier {goal_frontier}'
                )
                self.filter_frontiers()
                return

            self.get_logger().debug('Navigation goal accepted')
            self.current_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                partial(
                    self.nav_result_callback,
                    goal_id=goal_id,
                    goal_frontier=goal_frontier,
                )
            )
        except Exception as error:
            self.clear_navigation_state()
            self.get_logger().error(f'Error in goal response callback: {error}')

    def nav_result_callback(self, future, goal_id, goal_frontier):
        if goal_id != self.current_goal_id:
            self.get_logger().debug(
                f'Ignoring stale navigation result for goal id {goal_id}'
            )
            return
        try:
            result = future.result()
            self.get_logger().debug(
                'Navigation result received for frontier '
                f'{goal_frontier} with status {result.status}: {result.result}'
            )
        except Exception as error:
            self.get_logger().error(
                f'Error in navigation result callback: {error}'
            )
        finally:
            self.clear_navigation_state()
            self.filter_frontiers()


def main(args=None):
    rclpy.init(args=args)
    node = ScoreAndPostNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutdown requested (KeyboardInterrupt).')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
