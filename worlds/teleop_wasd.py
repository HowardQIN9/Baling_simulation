#!/usr/bin/env python3

import sys
import tty
import termios
import subprocess

TOPIC = "/model/simple_tractor/cmd_vel"

# Initial command
linear = 0.0
angular = 0.0

# Control parameters
LINEAR_STEP = 0.1
ANGULAR_STEP = 0.1

MAX_LINEAR = 1.5
MAX_ANGULAR = 0.5


def publish(linear_x, angular_z):
    """Send velocity command to Gazebo."""

    msg = (
        f"linear: {{x: {linear_x:.2f}}}, "
        f"angular: {{z: {angular_z:.2f}}}"
    )

    subprocess.run(
        [
            "gz", "topic",
            "-t", TOPIC,
            "-m", "gz.msgs.Twist",
            "-p", msg
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )


def get_key():
    """Read one key without pressing Enter."""

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    return key


print("""
==============================
 Tractor WASD Control
==============================

W : accelerate forward
S : accelerate backward

A : turn more left
D : turn more right

C : center steering
SPACE : stop
Q : quit

------------------------------
""")

try:

    while True:

        key = get_key().lower()

        if key == "w":
            linear += LINEAR_STEP
            linear = min(linear, MAX_LINEAR)

        elif key == "s":
            linear -= LINEAR_STEP
            linear = max(linear, -MAX_LINEAR)

        elif key == "a":
            angular += ANGULAR_STEP
            angular = min(angular, MAX_ANGULAR)

        elif key == "d":
            angular -= ANGULAR_STEP
            angular = max(angular, -MAX_ANGULAR)

        elif key == "c":
            angular = 0.0

        elif key == " ":
            linear = 0.0
            angular = 0.0

        elif key == "q":
            linear = 0.0
            angular = 0.0
            publish(linear, angular)
            print("\nStopped.")
            break

        else:
            continue

        publish(linear, angular)

        print(
            f"\rSpeed: {linear:+.2f} m/s   "
            f"Yaw command: {angular:+.2f} rad/s    ",
            end="",
            flush=True
        )

except KeyboardInterrupt:

    publish(0.0, 0.0)
    print("\nStopped.")
