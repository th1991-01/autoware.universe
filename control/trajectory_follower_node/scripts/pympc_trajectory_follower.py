#!/usr/bin/env python3
...

from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_planning_msgs.msg import Trajectory
from autoware_auto_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


def getYaw(orientation_xyzw):
    return R.from_quat(orientation_xyzw.reshape(-1, 4)).as_euler("xyz")[:, 2]


class PyMPCTrajectoryFollower(Node):
    def __init__(self):
        super().__init__("pympc_trajectory_follower")
        self.sub_trajectory_ = self.create_subscription(
            Trajectory,
            "/planning/scenario_planning/trajectory",
            #'input/trajectory',
            self.onTrajectory,
            1,
        )
        self.sub_trajectory_  # prevent unused variable warning

        self.sub_odometry_ = self.create_subscription(
            Odometry,
            "/localization/kinematic_state",
            #'input/kinematic_state',
            self.onOdometry,
            1,
        )
        self.sub_odometry_  # prevent unused variable warning

        self.sub_accel_ = self.create_subscription(
            AccelWithCovarianceStamped,
            "/localization/acceleration",
            #'input/acceleration',
            self.onAccel,
            1,
        )
        self.sub_accel_  # prevent unused variable warning

        self.sub_steering_ = self.create_subscription(
            SteeringReport,
            "/vehicle/status/steering_status",
            #'input/steering_status',
            self.onSteering,
            1,
        )
        self.sub_steering_  # prevent unused variable warning

        self.sub_operation_mode_ = self.create_subscription(
            OperationModeState,
            "/system/operation_mode/state",
            #'input/steering_status',
            self.onOperationMode,
            1,
        )
        self.sub_operation_mode_  # prevent unused variable warning

        self.control_cmd_pub_ = self.create_publisher(
            AckermannControlCommand,
            "/control/trajectory_follower/control_cmd",
            # 'output/control_cmd',
            1,
        )
        timer_period = 0.03  # 30ms
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self._present_trajectory = None
        self._present_kinematic_state = None
        self._present_acceleration = None
        self._present_steering_status = None
        self._present_operation_mode = None

    def onTrajectory(self, msg):
        self._present_trajectory = msg

    def onOdometry(self, msg):
        self._present_kinematic_state = msg

    def onAccel(self, msg):
        self._present_acceleration = msg

    def onSteering(self, msg):
        self._present_steering_status = msg

    def onOperationMode(self, msg):
        self._present_operation_mode = msg

    def timer_callback(self):
        if (self._present_trajectory is not None) and (self._present_kinematic_state is not None):
            self.pure_pursuit_control()

    def pure_pursuit_control(self):
        trajectory_position = []
        trajectory_orientation = []
        trajectory_longitudinal_velocity = []
        trajectory_lateral_velocity = []
        trajectory_acceleration = []
        trajectory_heading_rate = []
        points = self._present_trajectory.points
        for i in range(len(points)):
            trajectory_position.append(
                [points[i].pose.position.x, points[i].pose.position.y, points[i].pose.position.z]
            )
            trajectory_orientation.append(
                [
                    points[i].pose.orientation.x,
                    points[i].pose.orientation.y,
                    points[i].pose.orientation.z,
                    points[i].pose.orientation.w,
                ]
            )
            trajectory_longitudinal_velocity.append(points[i].longitudinal_velocity_mps)
            trajectory_lateral_velocity.append(points[i].lateral_velocity_mps)
            trajectory_acceleration.append(points[i].acceleration_mps2)
            trajectory_heading_rate.append(points[i].heading_rate_rps)
        trajectory_position = np.array(trajectory_position)
        trajectory_orientation = np.array(trajectory_orientation)

        present_position = np.array(
            [
                self._present_kinematic_state.pose.pose.position.x,
                self._present_kinematic_state.pose.pose.position.y,
                self._present_kinematic_state.pose.pose.position.z,
            ]
        )
        present_orientation = np.array(
            [
                self._present_kinematic_state.pose.pose.orientation.x,
                self._present_kinematic_state.pose.pose.orientation.y,
                self._present_kinematic_state.pose.pose.orientation.z,
                self._present_kinematic_state.pose.pose.orientation.w,
            ]
        )
        present_linear_velocity = np.array(
            [
                self._present_kinematic_state.twist.twist.linear.x,
                self._present_kinematic_state.twist.twist.linear.y,
                self._present_kinematic_state.twist.twist.linear.z,
            ]
        )
        present_angular_velocity = np.array(
            [
                self._present_kinematic_state.twist.twist.angular.x,
                self._present_kinematic_state.twist.twist.angular.y,
                self._present_kinematic_state.twist.twist.angular.z,
            ]
        )
        present_linear_acceleration = np.array(
            [
                self._present_acceleration.accel.accel.linear.x,
                self._present_acceleration.accel.accel.linear.y,
                self._present_acceleration.accel.accel.linear.z,
            ]
        )
        present_angular_acceleration = np.array(
            [
                self._present_acceleration.accel.accel.angular.x,
                self._present_acceleration.accel.accel.angular.y,
                self._present_acceleration.accel.accel.angular.z,
            ]
        )

        present_steering_tire_angle = self._present_steering_status.steering_tire_angle

        nearestIndex = ((trajectory_position - present_position) ** 2).sum(axis=1).argmin()

        # compute acc cmd
        longituginal_vel_err = (
            present_linear_velocity - trajectory_longitudinal_velocity[nearestIndex]
        )
        acc_kp = 0.5
        acc_lim = 2.0
        acc_cmd = np.clip(-acc_kp * longituginal_vel_err, -acc_lim, acc_lim)[0]

        # compute steer cmd
        nearest_trajectory_point_yaw = getYaw(trajectory_orientation[nearestIndex])
        cosyaw = np.cos(nearest_trajectory_point_yaw)
        sinyaw = np.sin(nearest_trajectory_point_yaw)
        diff_position = present_position - trajectory_position[nearestIndex]
        lat_err = -sinyaw * diff_position[0] + cosyaw * diff_position[1]
        yaw_err = getYaw(present_orientation) - nearest_trajectory_point_yaw
        while True:
            if yaw_err > np.pi:
                yaw_err -= 2.0 * np.pi
            if yaw_err < (-np.pi):
                yaw_err += 2.0 * np.pi
            if np.abs(yaw_err) < np.pi:
                break
        wheel_base = 4.0
        lookahead_time = 3.0
        min_lookahead = 3.0
        lookahead = min_lookahead + lookahead_time * np.abs(present_linear_velocity[0])
        steer_kp = 2.0 * wheel_base / (lookahead * lookahead)
        steer_kd = 2.0 * wheel_base / lookahead
        steer_lim = 0.6
        steer_cmd = np.clip(-steer_kp * lat_err - steer_kd * yaw_err, -steer_lim, steer_lim)[0]

        msg = AckermannControlCommand()
        msg.stamp = msg.lateral.stamp = msg.longitudinal.stamp = self.get_clock().now().to_msg()
        msg.longitudinal.speed = trajectory_longitudinal_velocity[nearestIndex]
        msg.longitudinal.acceleration = acc_cmd
        msg.lateral.steering_tire_angle = steer_cmd

        test_step_input_flag = False
        if test_step_input_flag:
            msg.longitudinal.speed = 0.25
            msg.longitudinal.acceleration = 1.0
            msg.lateral.steering_tire_angle = 0.3

        self.control_cmd_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    pympc_trajectory_follower = PyMPCTrajectoryFollower()

    rclpy.spin(pympc_trajectory_follower)

    pympc_trajectory_follower.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
