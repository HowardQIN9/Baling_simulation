"""Planar wheel-speed / IMU-heading integration, with no world pose input.

The first simulated tractor pose is known to be (0,0,0). The IMU is ideal in this
baseline; adding noise and a calibrated probabilistic filter is a later step.
"""
import math

REAR_AXLE_OFFSET = 1.321  # model origin is this far ahead of rear axle


def yaw_from_quaternion(q):
    return math.atan2(2*(q[3]*q[2]+q[0]*q[1]),1-2*(q[1]*q[1]+q[2]*q[2]))


class PlanarEstimator:
    def __init__(self):
        self.position = [0., 0., 0.]
        self.rear_position = None
        self.last_stamp = None
        self.last_yaw = 0.
        self.last_speed = 0.

    def update(self, stamp, raw_odometry, imu):
        q = imu["quaternion_xyzw"]
        yaw = yaw_from_quaternion(q)
        speed = raw_odometry["linear"][0]
        if self.rear_position is None:
            self.rear_position = [-REAR_AXLE_OFFSET*math.cos(yaw),
                                  -REAR_AXLE_OFFSET*math.sin(yaw)]
        if self.last_stamp is not None:
            dt = stamp-self.last_stamp
            if not 0 < dt < .25:
                raise ValueError("Wheel/IMU estimator received non-monotonic or stale odometry")
            self.rear_position[0] += .5*dt*(self.last_speed*math.cos(self.last_yaw)+speed*math.cos(yaw))
            self.rear_position[1] += .5*dt*(self.last_speed*math.sin(self.last_yaw)+speed*math.sin(yaw))
        self.position = [self.rear_position[0]+REAR_AXLE_OFFSET*math.cos(yaw),
                         self.rear_position[1]+REAR_AXLE_OFFSET*math.sin(yaw),0.]
        self.last_stamp, self.last_yaw, self.last_speed = stamp, yaw, speed
        return dict(raw_odometry, position=list(self.position), quaternion_xyzw=q,
                    source="integrated_wheel_speed_and_ideal_imu_heading")
