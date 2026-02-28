#!/usr/bin/env python3
"""
ROS2 Motor Driver Node
Drives 2 Moteus CAN controllers (joints 0-1) and 3 GPIO PWM motors (joints 2-4).
Subscribes to /velocity_controller/commands (Float64MultiArray, length 5).

Keyboard mapping (keyboard.py, NUM_JOINTS=5):
  1/q -> Moteus motor 0
  2/w -> Moteus motor 1
  3/e -> GPIO motor 0 (joint 2)
  4/r -> GPIO motor 1 (joint 3)
  5/t -> GPIO motor 2 (joint 4)

Hardware assumptions:
  - Moteus controllers on CAN bus, IDs 1 and 2 (set MOTEUS_IDS below)
  - GPIO PWM motors on pins defined in GPIO_PWM_PINS (BOARD numbering)
  - Running on an NVIDIA Jetson (Orin / Xavier / Nano)
  - Jetson.GPIO library installed: pip install Jetson.GPIO
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import asyncio
import threading

# ── Configuration ──────────────────────────────────────────────────────────────
NUM_MOTEUS   = 2          # number of Moteus CAN motors
NUM_GPIO     = 3          # number of GPIO PWM motors
NUM_JOINTS   = NUM_MOTEUS + NUM_GPIO  # total = 5

MOTEUS_IDS   = [1, 2]     # CAN IDs of Moteus controllers

# BOARD pin numbers for GPIO motor PWM signals (physical header pins)
# Common Jetson PWM-capable pins: 32, 33 (Orin/Xavier/Nano vary — check your pinout)
GPIO_PWM_PINS = [32, 33, 15]   # one pin per GPIO motor
GPIO_PWM_FREQ = 50             # Hz (standard servo/ESC frequency)

# Velocity scale: maps [-1, 1] command to Moteus velocity (rev/s)
MOTEUS_VEL_SCALE = 5.0

# Velocity to duty-cycle mapping for GPIO motors
# cmd in [-1, 1] -> duty cycle in [GPIO_DC_MIN, GPIO_DC_MAX]
GPIO_DC_MIN  = 5.0    # % duty cycle for full reverse
GPIO_DC_MID  = 7.5    # % duty cycle for stop
GPIO_DC_MAX  = 10.0   # % duty cycle for full forward
# ──────────────────────────────────────────────────────────────────────────────


def vel_to_duty(vel: float) -> float:
    """Map velocity [-1, 1] to PWM duty cycle."""
    vel = max(-1.0, min(1.0, vel))
    if vel >= 0:
        return GPIO_DC_MID + vel * (GPIO_DC_MAX - GPIO_DC_MID)
    else:
        return GPIO_DC_MID + vel * (GPIO_DC_MID - GPIO_DC_MIN)


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver_node')

        # ── GPIO setup ────────────────────────────────────────────────────────
        try:
            import Jetson.GPIO as GPIO
            self._GPIO = GPIO
            GPIO.setmode(GPIO.BOARD)
            self._pwm_channels = []
            for pin in GPIO_PWM_PINS:
                GPIO.setup(pin, GPIO.OUT)
                pwm = GPIO.PWM(pin, GPIO_PWM_FREQ)
                pwm.start(GPIO_DC_MID)  # neutral / stopped
                self._pwm_channels.append(pwm)
            self.get_logger().info(f"GPIO PWM initialised on pins {GPIO_PWM_PINS}")
        except ImportError:
            self.get_logger().warn("Jetson.GPIO not found – GPIO motors disabled (simulation mode)")
            self._GPIO = None
            self._pwm_channels = [None] * NUM_GPIO

        # ── Moteus async setup ────────────────────────────────────────────────
        try:
            import moteus
            self._moteus = moteus
            self._moteus_controllers = {}
            self._moteus_loop = asyncio.new_event_loop()
            self._moteus_thread = threading.Thread(
                target=self._moteus_loop.run_forever, daemon=True)
            self._moteus_thread.start()
            # Initialise controllers in the async loop
            future = asyncio.run_coroutine_threadsafe(
                self._init_moteus(), self._moteus_loop)
            future.result(timeout=5.0)
            self.get_logger().info(f"Moteus controllers initialised: IDs {MOTEUS_IDS}")
        except ImportError:
            self.get_logger().warn("moteus library not found – Moteus motors disabled (simulation mode)")
            self._moteus = None
            self._moteus_controllers = {}
            self._moteus_loop = None

        # ── Velocity state ────────────────────────────────────────────────────
        self._velocities = [0.0] * NUM_JOINTS

        # ── ROS subscriber ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self._cmd_callback,
            10)

        # ── Control loop timer (20 Hz) ─────────────────────────────────────
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f"motor_driver_node ready  "
            f"({NUM_MOTEUS} Moteus + {NUM_GPIO} GPIO, {NUM_JOINTS} joints total)")

    # ── Moteus async initialisation ───────────────────────────────────────────
    async def _init_moteus(self):
        for cid in MOTEUS_IDS:
            ctrl = self._moteus.Controller(id=cid)
            await ctrl.set_stop()           # clear any faults
            self._moteus_controllers[cid] = ctrl

    # ── ROS callback ─────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Float64MultiArray):
        n = min(len(msg.data), NUM_JOINTS)
        for i in range(n):
            self._velocities[i] = float(msg.data[i])

    # ── Main control loop ─────────────────────────────────────────────────────
    def _control_loop(self):
        # Moteus joints (0 .. NUM_MOTEUS-1)
        if self._moteus and self._moteus_loop:
            asyncio.run_coroutine_threadsafe(
                self._send_moteus_commands(), self._moteus_loop)

        # GPIO joints (NUM_MOTEUS .. NUM_JOINTS-1)
        for i in range(NUM_GPIO):
            joint_idx = NUM_MOTEUS + i
            vel = self._velocities[joint_idx]
            dc = vel_to_duty(vel)
            pwm = self._pwm_channels[i]
            if pwm is not None:
                pwm.ChangeDutyCycle(dc)
            else:
                # Simulation: just log non-zero commands
                if vel != 0.0:
                    self.get_logger().debug(
                        f"[SIM] GPIO motor {i}  vel={vel:.3f}  duty={dc:.1f}%")

    async def _send_moteus_commands(self):
        for i, cid in enumerate(MOTEUS_IDS):
            vel = self._velocities[i] * MOTEUS_VEL_SCALE
            ctrl = self._moteus_controllers.get(cid)
            if ctrl is None:
                continue
            try:
                await ctrl.set_position(
                    position=float('nan'),   # position NaN = velocity mode
                    velocity=vel,
                    query=False)
            except Exception as e:
                self.get_logger().error(f"Moteus ID {cid} error: {e}")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down motor_driver_node …")

        # Stop all Moteus motors
        if self._moteus and self._moteus_loop:
            async def _stop_all():
                for ctrl in self._moteus_controllers.values():
                    await ctrl.set_stop()
            future = asyncio.run_coroutine_threadsafe(
                _stop_all(), self._moteus_loop)
            try:
                future.result(timeout=2.0)
            except Exception:
                pass
            self._moteus_loop.call_soon_threadsafe(self._moteus_loop.stop)

        # Stop GPIO motors and clean up
        if self._GPIO:
            for pwm in self._pwm_channels:
                if pwm:
                    pwm.ChangeDutyCycle(GPIO_DC_MID)  # neutral before stop
                    pwm.stop()
            self._GPIO.cleanup()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
