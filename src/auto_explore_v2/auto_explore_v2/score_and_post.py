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
from std_srvs.srv import Empty, SetBool
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException

PATH_BLOCKED_OCCUPANCY_MIN = 51
PREFLIGHT_TIMEOUT_SEC = 10.0

# --- EXPLORATION TUNING PARAMETERS ---
# Penalty subtracted from a frontier's score if it fails the is_path_clear wall-clipping check.
# High values heavily stagger stubborn/blocked frontiers toward the end of exploration.
PATH_BLOCKED_PENALTY = 1000

# Penalty subtracted from a frontier's score if Nav2 fails to execute the path and physically times out.
TIMEOUT_PENALTY = 500

# Assumed robot speed (m/s) used to calculate dynamic timeouts. 
# Lower values give the robot proportionately more time to traverse long paths.
DYNAMIC_TIMEOUT_SPEED = 0.10

# Flat maximum time buffer (seconds) added to the dynamic timeout to allow for tight maneuvers and recovery spins.
DYNAMIC_TIMEOUT_BUFFER = 40.0
# -------------------------------------


class ScoreAndPostNode(Node):
    def __init__(self):
        super().__init__('score_and_post')
        self.get_logger().debug('score_and_post node has started')

        # State Flags
        self.exploration_active = True

        # Service
        self.toggle_service = self.create_service(
            SetBool,
            'toggle_exploration',
            self.toggle_callback
        )
        self.clear_blacklist_service = self.create_service(
            Empty,
            'clear_blacklist',
            self.clear_blacklist_callback
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
        self.frontier_attempts = {}
        self.timeout_attempts = {}

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

    def clear_blacklist_callback(self, request, response):
        self.frontier_attempts.clear()
        self.timeout_attempts.clear()
        self.get_logger().info('Map completion constraints relaxed. Blacklists and attempts wiped.')
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
        frontier_x, frontier_y = frontier[0], frontier[1]
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
                    frontier_key = self.frontier_key(self.active_goal_frontier)
                    self.timeout_attempts[frontier_key] = self.timeout_attempts.get(frontier_key, 0) + 1
                    self.get_logger().warning(f'Frontier {self.active_goal_frontier} timed out. Adding to timeout attempts penalty.')
                
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
        
        frontier_key = self.frontier_key(goal_frontier)
        attempts = self.frontier_attempts.get(frontier_key, 0)
        
        if not path.poses:
            self.frontier_attempts[frontier_key] = attempts + 1
            self.clear_navigation_state()
            self.get_logger().debug(
                'Nav2 could not produce a usable path for frontier cluster '
                f'{goal_frontier}. Incrementing attempt count.'
            )
            self.filter_frontiers()
            return

        path_is_clear, blocked_cells = self.is_path_clear(path)
        if not path_is_clear:
            self.frontier_attempts[frontier_key] = attempts + 1
            self.clear_navigation_state()
            self.get_logger().debug(
                'Manual path validation found blocked cells for frontier '
                f'cluster {goal_frontier}. Incrementing attempt count.'
            )
            self.filter_frontiers()
            return

        self.get_logger().debug(
            f'Computed path with {len(path.poses)} poses for frontier '
            f'{goal_frontier}; manual path validation passed'
        )
        
        path_length_meters = len(path.poses) * self.map_resolution

        self.goal_timeout_sec = (path_length_meters / DYNAMIC_TIMEOUT_SPEED) + DYNAMIC_TIMEOUT_BUFFER
        self.get_logger().debug(f'Dynamic timeout for local path length {path_length_meters:.1f}m set to {self.goal_timeout_sec:.1f}s')

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

        frontier_x, frontier_y = frontier[0], frontier[1]
        robot_x, robot_y = robot_position
        distance = (
            (frontier_x - robot_x) ** 2 + (frontier_y - robot_y) ** 2
        ) ** 0.5
        return distance

    def filter_frontiers(self):
        if not self.exploration_active:
            return

        if self.navigation_in_progress or self.preflight_in_progress:
            self.get_logger().debug(
                'Frontier attempt already in progress - skipping scoring'
            )
            return

        if not hasattr(self, 'frontiers') or not isinstance(self.frontiers, list) or not self.frontiers:
            if self.exploration_active:
                self.get_logger().info('ZERO valid frontiers remaining. Waiting for fresh frontiers from sensors...')
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
            frontier_key = str(tuple(frontier[:2]))
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
                
                attempts = self.frontier_attempts.get(frontier_key, 0)
                timeouts = self.timeout_attempts.get(frontier_key, 0)
                
                # Unify into a single competitive score
                frontier_size = frontier[2] if len(frontier) > 2 else 0
                size_bonus = frontier_size * 1.5  # Tuned down to 1.5 to balance nicely with distance

                # Dynamic Penalty: neutralizes the size_bonus and adds a baseline penalty to drop score smoothly
                dynamic_attempt_penalty = PATH_BLOCKED_PENALTY + size_bonus
                dynamic_timeout_penalty = TIMEOUT_PENALTY + size_bonus
                
                penalty = (-dynamic_attempt_penalty * attempts) + (-dynamic_timeout_penalty * timeouts)
                
                final_score = bfs_score + size_bonus - distance_score + penalty
                
                self.scored_frontiers[frontier_key] = final_score
            else:
                self.get_logger().warning(
                    f'Frontier {frontier_key} not found in BFS data; skipping.'
                )

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
        
        for frontier in ranked_frontiers:
            if frontier == str(self.last_goal_frontier):
                continue
            best_frontier = frontier
            break

        if best_frontier is None:
            self.get_logger().debug(
                'No new scored frontier is available for preflight. Waiting in idle.'
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
            from action_msgs.msg import GoalStatus
            
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info(
                    f'Successfully reached frontier {goal_frontier}.'
                )
            else:
                self.get_logger().warning(
                    'Navigation failed/aborted for frontier '
                    f'{goal_frontier} with status {result.status}'
                )
                # Terminal Strike: Nav2 failed execution. Let the penalty dynamically drop its priority.
                frontier_key = self.frontier_key(goal_frontier)
                attempts = self.frontier_attempts.get(frontier_key, 0)
                if attempts >= 2:
                    self.get_logger().error(f"Navigation to {goal_frontier} intrinsically failed after 3 attempts.")
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