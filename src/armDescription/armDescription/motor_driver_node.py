#!/usr/bin/env python3
"""
ROS2 Motor Driver Node — 3× GPIO PWM motors (ESC/servo, RC pulse widths)
==========================================================================
Pulse width mapping:
  1000 µs  →  full reverse
  1500 µs  →  neutral / stop  (sent on startup to arm ESCs)
  2000 µs  →  full forward

Subscribes to /velocity_controller/commands (Float64MultiArray, length 3).
Commands are floats in [-1.0 … +1.0].

Keyboard mapping (keyboard.py, NUM_JOINTS=3):
  1/q  →  motor 0  (pin 32)
  2/w  →  motor 1  (pin 33)
  3/e  →  motor 2  (pin 15  — see PWM note below)

──────────────────────────────────────────────────────────────────────────────
PWM PIN NOTES (Jetson Nano / Orin / Xavier)
──────────────────────────────────────────────────────────────────────────────
• Pins 32 and 33 are the only *hardware* PWM pins on the standard 40-pin header.
• Pin 15 is a regular GPIO and cannot do hardware PWM natively.
  For a real third ESC/servo use one of these alternatives:
    A) Use a PCA9685 I²C PWM board  (16 channels, recommended for 3+ motors)
    B) Orin/Xavier boards may expose additional PWM channels — check your pinout
  In simulation mode all three channels work fine (no real hardware needed).

This driver controls hardware channels through the Linux sysfs PWM interface
directly (nanosecond precision), bypassing the duty-cycle rounding in the
Jetson.GPIO Python wrapper.  Jetson.GPIO is still used for software PWM fallback.

Sysfs PWM chip mapping (Jetson Nano — adjust for your board):
  Pin 32  →  /sys/class/pwm/pwmchip0/pwm0
  Pin 33  →  /sys/class/pwm/pwmchip0/pwm2   (or pwmchip2/pwm0 on some images)
  Pin 15  →  NOT a hardware PWM; falls back to Jetson.GPIO software PWM
──────────────────────────────────────────────────────────────────────────────
"""

import os
import time
import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# ── Configuration ──────────────────────────────────────────────────────────────
NUM_JOINTS = 3

# RC pulse widths in microseconds
PWM_MIN_US     = 1000   # full reverse
PWM_NEUTRAL_US = 1500   # stop / neutral
PWM_MAX_US     = 2000   # full forward
PWM_FREQ_HZ    = 50     # standard RC frequency (20 ms period)

ESC_ARM_DURATION_S = 2.0   # seconds to hold neutral on startup (ESC arming)

# ── Sysfs PWM channel definitions ─────────────────────────────────────────────
# Each entry is the sysfs directory for that PWM channel.
# Adjust for your specific Jetson board / kernel image.
# Set to None to fall back to Jetson.GPIO software PWM for that pin.
SYSFS_PWM_DIRS = [
    "/sys/class/pwm/pwmchip0/pwm0",   # pin 32
    "/sys/class/pwm/pwmchip0/pwm2",   # pin 33  (try pwmchip2/pwm0 if this fails)
    None,                              # pin 15 — no hardware PWM; uses software PWM
]

# BOARD pin numbers (used only for software-PWM fallback channels)
GPIO_PWM_PINS = [32, 33, 15]

# Pre-compute constants
_PERIOD_NS  = int(1_000_000_000 / PWM_FREQ_HZ)    # 20 000 000 ns
_PERIOD_US  = 1_000_000.0 / PWM_FREQ_HZ            # 20 000 µs
_DC_NEUTRAL = (PWM_NEUTRAL_US / _PERIOD_US) * 100.0
_DC_MAX     = (PWM_MAX_US     / _PERIOD_US) * 100.0
_DC_MIN     = (PWM_MIN_US     / _PERIOD_US) * 100.0
# ──────────────────────────────────────────────────────────────────────────────


# ── Sysfs PWM wrapper ──────────────────────────────────────────────────────────

class SysfsPWM:
    """
    Controls a hardware PWM channel via the Linux sysfs interface.
    Uses nanosecond pulse widths for maximum precision.
    """

    def __init__(self, pwm_dir: str):
        self._dir          = pwm_dir
        self._period_path  = os.path.join(pwm_dir, "period")
        self._duty_path    = os.path.join(pwm_dir, "duty_cycle")
        self._enable_path  = os.path.join(pwm_dir, "enable")
        self._polarity_path = os.path.join(pwm_dir, "polarity")
        self._ok = False

    @staticmethod
    def _write(path: str, value: str) -> bool:
        try:
            with open(path, 'w') as f:
                f.write(value)
            return True
        except OSError:
            return False

    def setup(self, period_ns: int) -> bool:
        # Export channel if sysfs dir not yet present
        if not os.path.isdir(self._dir):
            chip_dir = os.path.dirname(self._dir)
            channel  = int(os.path.basename(self._dir).replace("pwm", ""))
            self._write(os.path.join(chip_dir, "export"), str(channel))
            time.sleep(0.15)

        if not os.path.isdir(self._dir):
            return False

        self._write(self._enable_path, "0")          # disable before config
        self._write(self._polarity_path, "normal")
        if not self._write(self._period_path, str(period_ns)):
            return False
        neutral_ns = PWM_NEUTRAL_US * 1000
        if not self._write(self._duty_path, str(neutral_ns)):
            return False
        if not self._write(self._enable_path, "1"):
            return False
        self._ok = True
        return True

    def set_pulse_us(self, pulse_us: float):
        if not self._ok:
            return
        clamped_us = max(PWM_MIN_US, min(PWM_MAX_US, pulse_us))
        self._write(self._duty_path, str(int(clamped_us * 1000)))

    def set_neutral(self):
        self.set_pulse_us(PWM_NEUTRAL_US)

    def stop(self):
        if self._ok:
            self.set_neutral()
            self._write(self._enable_path, "0")
        self._ok = False


# ── Velocity helpers ──────────────────────────────────────────────────────────

def vel_to_pulse_us(vel: float) -> float:
    """Map velocity [-1.0, +1.0]  →  RC pulse width in microseconds."""
    vel = max(-1.0, min(1.0, vel))
    if vel >= 0.0:
        return PWM_NEUTRAL_US + vel * (PWM_MAX_US - PWM_NEUTRAL_US)
    else:
        return PWM_NEUTRAL_US + vel * (PWM_NEUTRAL_US - PWM_MIN_US)


def vel_to_duty(vel: float) -> float:
    """Map velocity [-1.0, +1.0]  →  duty cycle % (for software PWM fallback)."""
    vel = max(-1.0, min(1.0, vel))
    if vel >= 0.0:
        return _DC_NEUTRAL + vel * (_DC_MAX - _DC_NEUTRAL)
    else:
        return _DC_NEUTRAL + vel * (_DC_NEUTRAL - _DC_MIN)


# ── ROS2 Node ──────────────────────────────────────────────────────────────────

class MotorDriverNode(Node):
    def __init__(self, dry_run: bool = False):
        super().__init__('motor_driver_node')

        self._dry_run    = dry_run
        self._sysfs_pwm  = []        # SysfsPWM or None per channel
        self._soft_pwm   = []        # Jetson.GPIO PWM or None per channel
        self._GPIO       = None
        self._velocities = [0.0] * NUM_JOINTS
        self._armed      = False

        if self._dry_run:
            self.get_logger().info(
                "*** DRY-RUN MODE — no hardware will be touched ***")
            self._sysfs_pwm = [None] * NUM_JOINTS
            self._soft_pwm  = [None] * NUM_JOINTS
        else:
            # ── Jetson.GPIO import (used for software PWM fallback only) ──────
            try:
                import Jetson.GPIO as GPIO
                self._GPIO = GPIO
                GPIO.setmode(GPIO.BOARD)
            except ImportError:
                self.get_logger().warn(
                    "Jetson.GPIO not found – software PWM fallback unavailable (simulation mode)")

            # ── Initialise each channel ───────────────────────────────────────
            for i in range(NUM_JOINTS):
                sysfs_dir = SYSFS_PWM_DIRS[i]
                pin       = GPIO_PWM_PINS[i]

                # 1) Try hardware sysfs PWM
                if sysfs_dir is not None:
                    ch = SysfsPWM(sysfs_dir)
                    if ch.setup(_PERIOD_NS):
                        self._sysfs_pwm.append(ch)
                        self._soft_pwm.append(None)
                        self.get_logger().info(
                            f"Motor {i}: ✓ hardware sysfs PWM  →  {sysfs_dir}")
                        continue
                    else:
                        self.get_logger().warn(
                            f"Motor {i}: sysfs PWM failed ({sysfs_dir}), "
                            f"falling back to software PWM on pin {pin}")

                # 2) Fallback: Jetson.GPIO software PWM
                self._sysfs_pwm.append(None)
                if self._GPIO is not None:
                    try:
                        self._GPIO.setup(pin, self._GPIO.OUT)
                        pwm = self._GPIO.PWM(pin, PWM_FREQ_HZ)
                        pwm.start(_DC_NEUTRAL)
                        self._soft_pwm.append(pwm)
                        self.get_logger().info(
                            f"Motor {i}: ⚠ software PWM on pin {pin} "
                            f"(less timing-precise)")
                        continue
                    except Exception as e:
                        self.get_logger().warn(
                            f"Motor {i}: software PWM on pin {pin} failed ({e}) "
                            f"→ simulation mode")

                # 3) Simulation
                self._soft_pwm.append(None)
                self.get_logger().warn(f"Motor {i}: simulation mode (no hardware)")

        # ── ESC arming: hold neutral for ESC_ARM_DURATION_S ──────────────────
        self.get_logger().info(
            f"Holding neutral ({PWM_NEUTRAL_US} µs) for "
            f"{ESC_ARM_DURATION_S:.1f} s to arm ESCs …")
        self._set_all_neutral()
        self._arm_timer = self.create_timer(ESC_ARM_DURATION_S, self._on_armed)

        # ── ROS subscriber ────────────────────────────────────────────────────
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/velocity_controller/commands',
            self._cmd_callback,
            10)

        # ── Control loop timer (50 Hz) ────────────────────────────────────────
        self.create_timer(0.02, self._control_loop)

        self.get_logger().info(
            f"motor_driver_node ready — {NUM_JOINTS} motors  "
            f"[{PWM_MIN_US} / {PWM_NEUTRAL_US} / {PWM_MAX_US} µs  "
            f"rev / neutral / fwd]")

    # ── ESC arm callback (fires once) ─────────────────────────────────────────
    def _on_armed(self):
        self._armed = True
        self._arm_timer.cancel()
        self.get_logger().info("ESCs armed — accepting velocity commands")

    # ── ROS callback ─────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Float64MultiArray):
        if not self._armed:
            return
        n = min(len(msg.data), NUM_JOINTS)
        for i in range(n):
            self._velocities[i] = float(msg.data[i])

    # ── Control loop ─────────────────────────────────────────────────────────
    def _control_loop(self):
        if not self._armed:
            return
        for i in range(NUM_JOINTS):
            vel      = self._velocities[i]
            pulse_us = vel_to_pulse_us(vel)

            if self._sysfs_pwm[i] is not None:
                self._sysfs_pwm[i].set_pulse_us(pulse_us)
            elif self._soft_pwm[i] is not None:
                self._soft_pwm[i].ChangeDutyCycle(vel_to_duty(vel))
            elif vel != 0.0 or self._dry_run:
                if vel != 0.0:
                    self.get_logger().info(
                        f"[DRY-RUN] motor {i}  vel={vel:+.3f}  pulse={pulse_us:.0f} µs")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _set_all_neutral(self):
        for i in range(NUM_JOINTS):
            if self._sysfs_pwm[i] is not None:
                self._sysfs_pwm[i].set_neutral()
            elif self._soft_pwm[i] is not None:
                self._soft_pwm[i].ChangeDutyCycle(_DC_NEUTRAL)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def destroy_node(self):
        self.get_logger().info("Shutting down — returning all motors to neutral …")
        self._set_all_neutral()
        time.sleep(0.1)

        for ch in self._sysfs_pwm:
            if ch is not None:
                ch.stop()

        if self._GPIO is not None:
            for pwm in self._soft_pwm:
                if pwm is not None:
                    pwm.stop()
            self._GPIO.cleanup()

        super().destroy_node()


def main(args=None):
    # Strip ROS args before argparse, then re-parse remaining
    import sys as _sys
    parser = argparse.ArgumentParser(description="Motor Driver Node")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Run without touching any hardware (for testing keyboard/topic flow)")
    known, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = MotorDriverNode(dry_run=known.dry_run)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()