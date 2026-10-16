#!/usr/bin/env python3
"""Combined driving / loader teleoperation using Gazebo Transport at 20 Hz."""
import select
import sys
import termios
import time
import tty

from gz.msgs10.double_pb2 import Double
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node


def main():
    if not sys.stdin.isatty():
        raise SystemExit("Run this controller in an interactive terminal.")
    node = Node()
    drive = node.advertise("/model/simple_tractor/cmd_vel", Twist)
    lift = node.advertise("/tractor/loader/lift/valve", Double)
    tilt = node.advertise("/tractor/loader/tilt/valve", Double)

    grip = node.advertise("/tractor/loader/grip/valve", Double)

    print("Waiting for Gazebo's lift, tilt, and grip controllers...", flush=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        if all(pub.has_connections() for pub in (lift, tilt, grip)):
            break
        time.sleep(0.1)
    else:
        missing = [name for name, pub in (("lift", lift), ("tilt", tilt), ("grip", grip))
                   if not pub.has_connections()]
        raise SystemExit(
            "No Gazebo subscriber for: " + ", ".join(missing) + ".\n"
            "The hydraulic plugin is missing, or the terminals use different GZ_PARTITION settings.\n"
            "Close the old Gazebo simulation and restart from the project directory:\n"
            "  bash scripts/run_sim.sh\n"
            "Then run this controller again.")

    def publish(speed, yaw, lift_value, tilt_value, grip_value):
        msg = Twist()
        msg.linear.x, msg.angular.z = speed, yaw
        drive.publish(msg)
        lift.publish(Double(data=lift_value))
        tilt.publish(Double(data=tilt_value))
        grip.publish(Double(data=grip_value))

    print("""Tractor + loader control (keep this terminal focused)
W/S: increase/decrease speed   A/D: increase left/right turn   C: steer straight
U/J: increase/decrease lift valve   I/K: increase/decrease tilt valve
O: open grip   P: close grip
N: neutral all loader valves, hold position / clamp force
SPACE: stop driving and neutral loader   Q or Ctrl-C: stop and exit

Commands latch: releasing a key does not stop motion. Use N or SPACE.
Loader valve settings change by 0.25 per press; range -1 to +1.
Positive lift raises the boom; positive tilt rolls the attachment carrier back.
Do not run another driving teleop at the same time.
""")
    old_settings = termios.tcgetattr(sys.stdin)
    speed = yaw = lift_value = tilt_value = grip_value = 0.0
    try:
        # cbreak retains normal Ctrl-C / signal behavior while reading single keys.
        tty.setcbreak(sys.stdin.fileno())
        while True:
            started = time.monotonic()
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == "q":
                    break
                if key == "w": speed = min(1.5, speed + 0.1)
                elif key == "s": speed = max(-1.5, speed - 0.1)
                elif key == "a": yaw = min(0.5, yaw + 0.1)
                elif key == "d": yaw = max(-0.5, yaw - 0.1)
                elif key == "c": yaw = 0.0
                elif key == "u": lift_value = min(1, lift_value + 0.25)
                elif key == "j": lift_value = max(-1, lift_value - 0.25)
                elif key == "i": tilt_value = min(1, tilt_value + 0.25)
                elif key == "k": tilt_value = max(-1, tilt_value - 0.25)
                elif key == "o": grip_value = -1.0
                elif key == "p": grip_value = 1.0
                elif key == "n": lift_value = tilt_value = grip_value = 0.0
                elif key == " ": speed = yaw = lift_value = tilt_value = grip_value = 0.0
            publish(speed, yaw, lift_value, tilt_value, grip_value)
            print(f"\rspeed={speed:+.1f} yaw={yaw:+.1f} "
                  f"lift={lift_value:+.2f} tilt={tilt_value:+.2f} grip={grip_value:+.1f} "
                  f"hydraulics={'CONNECTED' if all(p.has_connections() for p in (lift, tilt, grip)) else 'DISCONNECTED'}  ",
                  end="", flush=True)
            time.sleep(max(0, 0.05 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        for _ in range(3):
            publish(0, 0, 0, 0, 0)
            time.sleep(0.05)
        print("\nStopped. Loader valves neutral.")


if __name__ == "__main__":
    main()
