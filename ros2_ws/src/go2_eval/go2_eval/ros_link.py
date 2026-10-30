"""ROS 2 side of the evaluation: talks to go2_policy_runner's policy_controller.

  * /cmd_vel                              velocity command of the scenario (heading hold, schedule)
  * /<controller>/set_mode                FSM transitions (stand, walk, passive)
  * /<controller>/state                   active mode, mode time, health (lowstate rate)
  * /<controller>/policy_step             raw actions of every policy step (for the action rate)

The DDS environment (RMW_IMPLEMENTATION, ROS_DOMAIN_ID, CYCLONEDDS_URI) must be set before this module is
imported (evaluate.py does it).
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from go2_policy_runner_interfaces.msg import ControllerState, PolicyStep
from go2_policy_runner_interfaces.srv import SetMode


class RunnerLink:
    def __init__(self, controller: str = "policy_controller", cmd_vel_topic: str = "/cmd_vel"):
        rclpy.init()
        self.node = rclpy.create_node("go2_eval")
        self.controller = controller
        self.cmd_pub = self.node.create_publisher(Twist, cmd_vel_topic, 10)
        self.mode_cli = self.node.create_client(SetMode, f"/{controller}/set_mode")
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(ControllerState, f"/{controller}/state", self._on_state, latched)
        self.node.create_subscription(PolicyStep, f"/{controller}/policy_step", self._on_step, 50)

        self._lock = threading.Lock()
        self._state: ControllerState | None = None
        self._state_time = 0.0
        self._prev_action: list[float] | None = None
        self._rates: list[float] = []
        self._last_rate = 0.0
        self.policy_steps = 0

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin = threading.Thread(target=self.executor.spin, daemon=True)
        self._spin.start()

    # ---- callbacks -------------------------------------------------------------------------------------

    def _on_state(self, msg: ControllerState):
        with self._lock:
            self._state = msg
            self._state_time = time.monotonic()

    def _on_step(self, msg: PolicyStep):
        action = list(msg.action)
        with self._lock:
            # evaluate.py: |a_t - a_(t-1)|, with a zero previous action at the start of the episode
            prev = self._prev_action if (msg.step > 0 and self._prev_action is not None) else [0.0] * len(action)
            if len(prev) == len(action):
                rate = math.sqrt(sum((a - b) ** 2 for a, b in zip(action, prev)))
                self._rates.append(rate)
                self._last_rate = rate
            self._prev_action = action
            self.policy_steps += 1

    # ---- API -------------------------------------------------------------------------------------------

    def state(self) -> ControllerState | None:
        with self._lock:
            return self._state

    def state_age(self) -> float:
        with self._lock:
            return time.monotonic() - self._state_time if self._state is not None else math.inf

    def take_action_rate(self) -> float:
        """Mean action rate of the policy steps received since the last call (the last one if none)."""
        with self._lock:
            rates, self._rates = self._rates, []
            return sum(rates) / len(rates) if rates else self._last_rate

    def reset_actions(self):
        """Start of an episode: forget the actions of the previous one (policy_steps counts this episode)."""
        with self._lock:
            self._rates, self._last_rate, self._prev_action = [], 0.0, None
            self.policy_steps = 0

    def cmd(self, vx: float, vy: float, wz: float):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(vx), float(vy), float(wz)
        self.cmd_pub.publish(msg)

    def set_mode(self, mode: str, timeout: float = 3.0) -> tuple[bool, str]:
        if not self.mode_cli.wait_for_service(timeout_sec=timeout):
            return False, f"service /{self.controller}/set_mode not available"
        req = SetMode.Request()
        req.mode = mode
        future = self.mode_cli.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return False, "set_mode timed out"
            time.sleep(0.005)
        res = future.result()
        return bool(res.success), str(res.message)

    def set_mode_async(self, mode: str):
        req = SetMode.Request()
        req.mode = mode
        return self.mode_cli.call_async(req)

    def wait_for_service(self, timeout: float) -> bool:
        return self.mode_cli.wait_for_service(timeout_sec=timeout)

    def shutdown(self):
        try:
            self.cmd(0.0, 0.0, 0.0)
            self.executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
