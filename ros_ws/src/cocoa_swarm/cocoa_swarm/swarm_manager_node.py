"""
SwarmManagerNode
================
ROS2 Action Server that receives a SwarmCommand goal and orchestrates multiple
individual drone command actions in parallel.

Action served:  swarm_command  (cocoa_msgs/action/SwarmCommand)
Actions called: /{drone_id}/drone_command  per active drone
                (action_server_node instances, remapped at launch)

Collision-avoidance strategy: altitude separation via ConflictResolver.
  → drone[0] flies at z=1.0 m, drone[1] at z=1.5 m to the same (x, y) target.

Parallel execution pattern:
  - send_goal_async() + add_done_callback() — callbacks driven by the executor
  - polling loop uses time.sleep(dt) (NOT asyncio.sleep) because rclpy's
    MultiThreadedExecutor does not guarantee a running asyncio event loop in
    the callback thread.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped

from cocoa_msgs.action import SwarmCommand, DroneCommand
from cocoa_swarm.swarm_model import SwarmModel
from cocoa_swarm.coordination import ConflictResolver
from cocoa_swarm.formation import FormationPlanner


class SwarmManagerNode(Node):
    """
    Swarm Manager — coordinates multiple drones toward a common target.

    Lifecycle:
        1. Accept SwarmCommand goal (drone_ids, target_x/y, target_z_per_drone)
        2. Resolve altitudes (use override or ConflictResolver defaults)
        3. For command_type == 'goto_swarm':
              a. Takeoff all drones to their assigned altitudes  (parallel)
              b. Goto target (x, y, assigned_z) for each drone   (parallel)
        4. Return combined success/failure result
    """

    def __init__(self):
        super().__init__('swarm_manager_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('drone_ids', ['cf231', 'cf232'])
        self.declare_parameter('base_z', 1.0)
        self.declare_parameter('z_separation', 0.5)
        self.declare_parameter('action_timeout', 60.0)
        self.declare_parameter('feedback_rate', 5.0)

        self.drone_ids: list[str] = self.get_parameter('drone_ids').value
        base_z: float = self.get_parameter('base_z').value
        z_sep: float = self.get_parameter('z_separation').value
        self.action_timeout: float = self.get_parameter('action_timeout').value
        self.feedback_rate: float = self.get_parameter('feedback_rate').value
        
        self.declare_parameter('inspection_radius', 1.5)
        self.inspection_radius: float = self.get_parameter('inspection_radius').value

        # Formation planner — uses EKG boundaries to position drones
        self.formation_planner = FormationPlanner(
            inspection_radius=self.inspection_radius
        )
        n_objects = len(self.formation_planner.ekg.get_all_objects())
        self.get_logger().info(f'[SwarmManager] FormationPlanner loaded with {n_objects} EKG objects.')

        # ── Callback group (allow concurrent action server + clients) ─────────
        self.cbg = ReentrantCallbackGroup()

        # ── Swarm model (live pose tracking) ─────────────────────────────────
        self.swarm_model = SwarmModel(self, self.drone_ids)

        # ── Conflict resolver (altitude separation) ───────────────────────────
        self.conflict_resolver = ConflictResolver(base_z=base_z, separation=z_sep)

        # ── Per-drone action clients: /{drone_id}/drone_command ───────────────
        self.drone_clients: dict[str, ActionClient] = {}
        for did in self.drone_ids:
            client = ActionClient(
                self,
                DroneCommand,
                f'/{did}/drone_command',
                callback_group=self.cbg,
            )
            self.drone_clients[did] = client
            self.get_logger().info(f'[SwarmManager] Created client for /{did}/drone_command')

        # ── Swarm action server ───────────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            SwarmCommand,
            'swarm_command',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cbg,
        )

        self.get_logger().info(
            f'[SwarmManager] Ready. Managing drones: {self.drone_ids}'
        )

    # =========================================================================
    # Goal / Cancel callbacks
    # =========================================================================

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            f'[SwarmManager] Received SwarmCommand: {goal_request.command_type}'
            f' for drones {list(goal_request.drone_ids)}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('[SwarmManager] Cancel requested')
        return CancelResponse.ACCEPT

    # =========================================================================
    # Main execute callback
    # =========================================================================

    async def _execute_callback(self, goal_handle):
        """Orchestrate the swarm command."""
        req = goal_handle.request
        drone_ids: list[str] = list(req.drone_ids)
        target_x: float = req.target_x
        target_y: float = req.target_y
        override_zs: list[float] = list(req.target_z_per_drone)
        cmd: str = req.command_type.lower()

        self.get_logger().info(
            f'[SwarmManager] Executing "{cmd}" → target=({target_x}, {target_y}) '
            f'for drones={drone_ids}'
        )

        # Resolve altitudes
        alt_map: dict[str, float] = self.conflict_resolver.assign_altitudes_for(
            drone_ids, override_zs if override_zs else None
        )
        self.get_logger().info(f'[SwarmManager] Altitude map: {alt_map}')

        # Wait for all action servers
        for did in drone_ids:
            client = self.drone_clients.get(did)
            if client is None:
                self.get_logger().error(f'[SwarmManager] No client for {did}')
                goal_handle.abort()
                return self._make_result(False, drone_ids, ['no_client'] * len(drone_ids))

            self.get_logger().info(f'[SwarmManager] Waiting for /{did}/drone_command ...')
            if not client.wait_for_server(timeout_sec=15.0):
                self.get_logger().error(f'[SwarmManager] /{did}/drone_command not available')
                goal_handle.abort()
                return self._make_result(False, drone_ids, ['server_unavailable'] * len(drone_ids))

        # ── Route by command type ─────────────────────────────────────────────
        if cmd == 'goto_swarm':
            success, results = await self._execute_goto_swarm(
                goal_handle, drone_ids, alt_map, target_x, target_y
            )
        elif cmd == 'takeoff_swarm':
            success, results = await self._execute_takeoff_swarm(
                goal_handle, drone_ids, alt_map
            )
        elif cmd == 'land_swarm':
            success, results = await self._execute_land_swarm(
                goal_handle, drone_ids
            )
        else:
            self.get_logger().error(f'[SwarmManager] Unknown command: {cmd}')
            goal_handle.abort()
            return self._make_result(False, drone_ids, [f'unknown:{cmd}'] * len(drone_ids))

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return self._make_result(success, drone_ids, results)

    # =========================================================================
    # goto_swarm: takeoff all → goto all
    # =========================================================================

    async def _execute_goto_swarm(
        self,
        goal_handle,
        drone_ids: list[str],
        alt_map: dict[str, float],
        target_x: float,
        target_y: float,
    ):
        """Step 1: Takeoff all to assigned altitude. Step 2: Goto target."""

        # Step 1 — Takeoff
        self.get_logger().info('[SwarmManager] Step 1: Takeoff (parallel)')
        takeoff_cmds = [
            (did, self._build_goal('takeoff', 0.0, 0.0, alt_map[did], 0.0, 0.0))
            for did in drone_ids
        ]
        takeoff_results = await self._send_parallel_goals(takeoff_cmds, goal_handle)
        if not all(r.get('success') for r in takeoff_results.values()):
            failed = [d for d, r in takeoff_results.items() if not r.get('success')]
            self.get_logger().error(f'[SwarmManager] Takeoff failed for: {failed}')
            msgs = [takeoff_results[d].get('message', 'fail') for d in drone_ids]
            return False, msgs

        self.get_logger().info(f'[SwarmManager] Step 2: Navigate to target (parallel)')
        # Step 2 — Compute formation positions via FormationPlanner
        waypoints = self.formation_planner.compute_inspection_positions(
            target_x, target_y, len(drone_ids)
        )
        obj_name = waypoints[0].object_name if waypoints else 'unknown'
        self.get_logger().info(
            f'[SwarmManager] Target: {obj_name}. '
            f'Formation positions: {[(f"{wp.x:.2f}", f"{wp.y:.2f}") for wp in waypoints]}'
        )

        # Send navigate_to (with path planner) to each drone in parallel
        # The per-drone ActionServer handles obstacle avoidance internally
        nav_cmds = []
        for did, wp in zip(drone_ids, waypoints):
            nav_cmds.append(
                (did, self._build_goal('navigate_to', wp.x, wp.y, alt_map[did], wp.yaw, 0.0))
            )

        nav_results = await self._send_parallel_goals(
            nav_cmds, goal_handle, publish_feedback=True, drone_ids=drone_ids
        )

        success = all(r.get('success') for r in nav_results.values())
        msgs = [nav_results[d].get('message', 'fail') for d in drone_ids]
        return success, msgs

    # =========================================================================
    # takeoff_swarm
    # =========================================================================

    async def _execute_takeoff_swarm(self, goal_handle, drone_ids, alt_map):
        cmds = [
            (did, self._build_goal('takeoff', 0.0, 0.0, alt_map[did], 0.0, 0.0))
            for did in drone_ids
        ]
        results = await self._send_parallel_goals(cmds, goal_handle)
        success = all(r.get('success') for r in results.values())
        msgs = [results[d].get('message', 'fail') for d in drone_ids]
        return success, msgs

    # =========================================================================
    # land_swarm
    # =========================================================================

    async def _execute_land_swarm(self, goal_handle, drone_ids):
        cmds = [
            (did, self._build_goal('land', 0.0, 0.0, 0.0, 0.0, 0.0))
            for did in drone_ids
        ]
        results = await self._send_parallel_goals(cmds, goal_handle)
        success = all(r.get('success') for r in results.values())
        msgs = [results[d].get('message', 'fail') for d in drone_ids]
        return success, msgs

    # =========================================================================
    # Parallel goal helper
    # =========================================================================

    async def _send_parallel_goals(
        self,
        commands: list[tuple],
        goal_handle,
        publish_feedback: bool = False,
        drone_ids: list[str] | None = None,
    ) -> dict[str, dict]:
        """
        Send multiple DroneCommand goals in parallel.

        Uses send_goal_async() + done callbacks (processed by the executor).
        Polls events with time.sleep(dt) — asyncio.sleep is not available in
        rclpy's MultiThreadedExecutor callback threads (no running event loop).

        Returns: {drone_id: {"success": bool, "message": str}}
        """
        results: dict[str, dict] = {}
        events: dict[str, threading.Event] = {}

        for did, goal_msg in commands:
            events[did] = threading.Event()
            results[did] = {'success': False, 'message': 'pending'}

            # Closures must capture did by value
            def _make_goal_cb(d):
                def _goal_response_cb(future):
                    gh = future.result()
                    if not gh.accepted:
                        results[d] = {'success': False, 'message': 'goal_rejected'}
                        events[d].set()
                        return

                    def _make_result_cb(dd):
                        def _result_cb(res_future):
                            res = res_future.result().result
                            results[dd] = {
                                'success': res.success,
                                'message': res.message,
                            }
                            events[dd].set()
                        return _result_cb

                    result_future = gh.get_result_async()
                    result_future.add_done_callback(_make_result_cb(d))

                return _goal_response_cb

            send_future = self.drone_clients[did].send_goal_async(goal_msg)
            send_future.add_done_callback(_make_goal_cb(did))

        # Poll until all drones have completed
        fb_counter = 0
        while not all(e.is_set() for e in events.values()):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('[SwarmManager] Goal cancelled during execution')
                goal_handle.canceled()
                return results

            # Optionally publish aggregate feedback
            if publish_feedback and drone_ids:
                fb_counter += 1
                if fb_counter % int(self.feedback_rate) == 0:
                    self._publish_swarm_feedback(goal_handle, drone_ids)

            # Use time.sleep instead of asyncio.sleep: rclpy's
            # MultiThreadedExecutor does not provide a running asyncio event
            # loop in the callback thread, so await asyncio.sleep() raises
            # RuntimeError. time.sleep() is safe here because threading.Events
            # are used (not asyncio Futures) and other threads continue.
            time.sleep(1.0 / self.feedback_rate)

        return results

    # =========================================================================
    # Feedback helper
    # =========================================================================

    def _publish_swarm_feedback(self, goal_handle, drone_ids: list[str]):
        """Publish aggregate position feedback for all drones."""
        feedback = SwarmCommand.Feedback()
        feedback.drone_ids = drone_ids
        states = [self.swarm_model.get_state(d) for d in drone_ids]
        feedback.current_x = [float(s.get('x', 0.0)) for s in states]
        feedback.current_y = [float(s.get('y', 0.0)) for s in states]
        feedback.current_z = [float(s.get('z', 0.0)) for s in states]
        feedback.distance_remaining = [0.0] * len(drone_ids)  # simplified
        feedback.status = ['flying'] * len(drone_ids)
        goal_handle.publish_feedback(feedback)

    # =========================================================================
    # Utilities
    # =========================================================================

    @staticmethod
    def _build_goal(
        command_type: str,
        x: float, y: float, z: float,
        yaw: float, duration: float,
    ) -> DroneCommand.Goal:
        goal = DroneCommand.Goal()
        goal.command_type = command_type
        goal.target_x = float(x)
        goal.target_y = float(y)
        goal.target_z = float(z)
        goal.yaw = float(yaw)
        goal.duration = float(duration)
        return goal

    @staticmethod
    def _make_result(
        success: bool,
        drone_ids: list[str],
        messages: list[str],
    ) -> SwarmCommand.Result:
        result = SwarmCommand.Result()
        result.success = success
        result.drone_results = messages
        return result


# =============================================================================
# Entry point
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SwarmManagerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
