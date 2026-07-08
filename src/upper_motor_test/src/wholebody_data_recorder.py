#!/usr/bin/env python3
"""
Data Recorder Node for Robot Testing
- Cache latest topic data in callbacks
- Snapshot synchronized rows at fixed control-feedback frequency
- Save CSV and plot on exit
"""

import os
import csv
import math
from datetime import datetime
from collections import defaultdict

# from build.vectornav_msgs.rosidl_generator_py.vectornav_msgs import msg
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from interfaces.msg import CustomJointState
from vectornav_msgs.msg import CommonGroup
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped


class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # =============================
        # Parameters
        # =============================
        self.record_dt = self.declare_parameter('record_dt', 0.02).value  # 50 Hz
        self.num_joints = self.declare_parameter('num_joints', 10).value

        # =============================
        # Output path
        # =============================
        self.results_dir = os.path.expanduser("~/ros2_ws/robot_test_data")
        os.makedirs(self.results_dir, exist_ok=True)

        self.timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_file = os.path.join(self.results_dir, f"robot_data_{self.timestamp}.csv")

        # =============================
        # Data storage
        # =============================
        self.data = defaultdict(list)
        self.time_start = None

        # =============================
        # Joint names
        # =============================
        self.joint_names = [
            "R_hip_joint",
            "R_hip2_joint",
            "R_thigh_joint",
            "R_calf_joint",
            "L_hip_joint",
            "L_hip2_joint",
            "L_thigh_joint",
            "L_calf_joint",
            "L_toe_joint",
            "R_toe_joint",
        ]

        # =============================
        # Latest cached signals
        # =============================
        self.latest_joint_pos = [0.0] * self.num_joints
        self.latest_joint_vel = [0.0] * self.num_joints
        self.latest_joint_effort = [0.0] * self.num_joints

        self.latest_torque_gravity_compensate = [0.0] * self.num_joints

        self.latest_cmd_pos = [0.0] * self.num_joints
        self.latest_cmd_kp = [0.0] * self.num_joints
        self.latest_cmd_kd = [0.0] * self.num_joints
        self.latest_cmd_effort = [0.0] * self.num_joints

        self.latest_torque_fb = [0.0] * self.num_joints

        self.latest_proj_g = [0.0, 0.0, -1.0]
        self.latest_wb = [0.0, 0.0, 0.0]
        self.latest_quat = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z
        self.latest_heading_angle = 0.0

        # Raw IMU data from VectorNav
        self.latest_imu_raw_acc = [0.0, 0.0, 0.0]    # msg.imu_accel
        self.latest_imu_raw_gyro = [0.0, 0.0, 0.0]   # msg.imu_rate

        self.latest_base_height_fb = 0.0
        self.latest_base_height_cmd = 0.0

        self.latest_estimated_grf_paper_left = [0.0, 0.0, 0.0]
        self.latest_estimated_grf_paper_right = [0.0, 0.0, 0.0]
        self.latest_estimated_contact_state = [0.0, 0.0]
        self.latest_contact_score = [0.0, 0.0]

        self.has_joint_state = False
        self.has_joint_cmd = False
        self.has_imu = False
        self.has_torque_fb = False
        self.has_base_height_fb = False
        self.has_base_height_cmd = False
        self.has_gravity_compensate = False
        self.has_estimated_contact_state = False
        self.has_estimated_grf_paper_left = False
        self.has_estimated_grf_paper_right = False

        # =============================
        # IMU state machine variables
        # =============================
        self.imu_state = 0
        self.imu_signal = 0
        self.imu_start_time = None
        self.qx_pi = Rotation.from_rotvec([math.pi, 0, 0])
        self.q_glob_init = None

        # =============================
        # Subscribers
        # =============================
        self.sub_joint_state = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        self.sub_imu_data = self.create_subscription(
            CommonGroup, 'vectornav/raw/common', self.imu_callback, 10)

        self.sub_joint_cmd = self.create_subscription(
            CustomJointState, '/joint_cmds', self.joint_cmd_callback, 10)

        self.sub_torque_fb = self.create_subscription(
            JointState, '/joint_states_fb_torque', self.torque_fb_callback, 10)

        self.sub_base_height_fb = self.create_subscription(
            Float32, '/base_height', self.base_height_fb_callback, 10)

        self.sub_base_height_cmd = self.create_subscription(
            Float32, '/base_height_cmd', self.base_height_cmd_callback, 10)
        
        self.sub_torque_gravity_compensate = self.create_subscription(
            JointState, '/torque_gravity_compensate', self.torque_gravity_compensate_callback, 10
        )

        self.sub_estimated_grf_paper_left = self.create_subscription(
            Vector3Stamped, '/estimated_grf_paper_left', self.estimated_grf_paper_left_callback, 10
        )

        self.sub_estimated_grf_paper_right = self.create_subscription(
            Vector3Stamped, '/estimated_grf_paper_right', self.estimated_grf_paper_right_callback, 10
        )

        self.sub_estimated_contact_state = self.create_subscription(
            JointState, '/estimated_contact_state', self.estimated_contact_state_callback, 10
        )

        # =============================
        # Fixed-rate recorder timer
        # =============================
        self.record_timer = self.create_timer(self.record_dt, self.record_snapshot)

        self.get_logger().info(
            f"Data Recorder started. record_dt={self.record_dt:.6f}s "
            f"({1.0/self.record_dt:.1f} Hz), saving to {self.csv_file}"
        )

    # =========================================================
    # Helper
    # =========================================================
    def _resize_or_pad(self, values, size):
        values = list(values)
        if len(values) >= size:
            return values[:size]
        return values + [0.0] * (size - len(values))

    def _now_sec(self):
        now = self.get_clock().now().to_msg()
        return now.sec + now.nanosec / 1e9

    # =========================================================
    # Callbacks: only cache latest values
    # =========================================================
    def estimated_grf_paper_left_callback(self, msg: Vector3Stamped):
        self.latest_estimated_grf_paper_left = [msg.vector.x, msg.vector.y, msg.vector.z]
        self.has_estimated_grf_paper_left = True

    def estimated_grf_paper_right_callback(self, msg: Vector3Stamped):
        self.latest_estimated_grf_paper_right = [msg.vector.x, msg.vector.y, msg.vector.z]
        self.has_estimated_grf_paper_right = True

    def estimated_contact_state_callback(self, msg: JointState):
        self.latest_estimated_contact_state = [msg.position[0], msg.position[1]]
        self.latest_contact_score = [msg.effort[0], msg.effort[1]]
        self.has_estimated_contact_state = True

    def joint_state_callback(self, msg: JointState):
        self.latest_joint_pos = self._resize_or_pad(msg.position, self.num_joints)
        self.latest_joint_vel = self._resize_or_pad(msg.velocity, self.num_joints)
        self.latest_joint_effort = self._resize_or_pad(msg.effort[:self.num_joints], self.num_joints)
        self.has_joint_state = True

    def joint_cmd_callback(self, msg: CustomJointState):
        self.latest_cmd_pos = self._resize_or_pad(msg.state.position, self.num_joints)
        self.latest_cmd_kp = self._resize_or_pad(msg.kp, self.num_joints)
        self.latest_cmd_kd = self._resize_or_pad(msg.kd, self.num_joints)
        self.latest_cmd_effort = self._resize_or_pad(msg.state.effort, self.num_joints)
        self.has_joint_cmd = True

    def torque_fb_callback(self, msg: JointState):
        self.latest_torque_fb = self._resize_or_pad(msg.effort[:self.num_joints], self.num_joints)
        self.has_torque_fb = True

    def base_height_fb_callback(self, msg: Float32):
        self.latest_base_height_fb = float(msg.data)
        self.has_base_height_fb = True

    def base_height_cmd_callback(self, msg: Float32):
        self.latest_base_height_cmd = float(msg.data)
        self.has_base_height_cmd = True
    
    def torque_gravity_compensate_callback(self, msg: JointState):
        self.latest_torque_gravity_compensate = self._resize_or_pad(msg.effort[:self.num_joints], self.num_joints)
        self.has_gravity_compensate = True

    def imu_callback(self, msg: CommonGroup):
        # Check NaN
        ang_nan = (
            np.isnan(msg.angularrate.x) or
            np.isnan(msg.angularrate.y) or
            np.isnan(msg.angularrate.z)
        )
        quat_nan = (
            np.isnan(msg.quaternion.x) or
            np.isnan(msg.quaternion.y) or
            np.isnan(msg.quaternion.z) or
            np.isnan(msg.quaternion.w)
        )
        imu_acc_nan = (
            np.isnan(msg.imu_accel.x) or
            np.isnan(msg.imu_accel.y) or
            np.isnan(msg.imu_accel.z)
        )
        imu_rate_nan = (
            np.isnan(msg.imu_rate.x) or
            np.isnan(msg.imu_rate.y) or
            np.isnan(msg.imu_rate.z)
        )
        if ang_nan or quat_nan or imu_acc_nan or imu_rate_nan:
            return

        # Check Inf
        ang_inf = (
            np.isinf(msg.angularrate.x) or
            np.isinf(msg.angularrate.y) or
            np.isinf(msg.angularrate.z)
        )
        quat_inf = (
            np.isinf(msg.quaternion.x) or
            np.isinf(msg.quaternion.y) or
            np.isinf(msg.quaternion.z) or
            np.isinf(msg.quaternion.w)
        )
        imu_acc_inf = (
            np.isinf(msg.imu_accel.x) or
            np.isinf(msg.imu_accel.y) or
            np.isinf(msg.imu_accel.z)
        )
        imu_rate_inf = (
            np.isinf(msg.imu_rate.x) or
            np.isinf(msg.imu_rate.y) or
            np.isinf(msg.imu_rate.z)
        )
        if ang_inf or quat_inf or imu_acc_inf or imu_rate_inf:
            return

        # init state machine
        if self.imu_state == 0 and self.imu_signal == 0:
            self.imu_signal = 1
            return

        elif self.imu_state == 0 and self.imu_signal == 1:
            if self.imu_start_time is None:
                self.imu_start_time = datetime.now()
            elapsed = (datetime.now() - self.imu_start_time).total_seconds()
            if elapsed > 1.0:
                self.imu_state = 1
                self.imu_signal = 0
            return

        elif self.imu_state == 1 and self.imu_signal == 0:
            q_imu_init = Rotation.from_quat([
                msg.quaternion.x,
                msg.quaternion.y,
                msg.quaternion.z,
                msg.quaternion.w
            ])
            q_robot_init = q_imu_init * self.qx_pi
            euler_angles = q_robot_init.as_euler('xyz')
            yaw = euler_angles[2]

            self.q_glob_init = Rotation.from_rotvec([0, 0, yaw]) * self.qx_pi
            self.imu_signal = 1
            return

        # normal case
        q_imu = Rotation.from_quat([
            msg.quaternion.x,
            msg.quaternion.y,
            msg.quaternion.z,
            msg.quaternion.w
        ])

        q_robot = self.q_glob_init.inv() * q_imu * self.qx_pi

        w_imu = np.array([msg.angularrate.x, msg.angularrate.y, msg.angularrate.z])
        w_robot = self.qx_pi.inv().apply(w_imu)

        # Raw IMU data, directly from VectorNav message
        self.latest_imu_raw_acc = [
            float(msg.imu_accel.x),
            float(msg.imu_accel.y),
            float(msg.imu_accel.z),
        ]

        self.latest_imu_raw_gyro = [
            float(msg.imu_rate.x),
            float(msg.imu_rate.y),
            float(msg.imu_rate.z),
        ]

        g_world = np.array([0.0, 0.0, -1.0])
        g_body = q_robot.inv().apply(g_world)

        quat_array = q_robot.as_quat()  # [x, y, z, w]
        x, y, z, w = quat_array[0], quat_array[1], quat_array[2], quat_array[3]

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        heading_angle = math.atan2(siny_cosp, cosy_cosp)

        self.latest_proj_g = [float(g_body[0]), float(g_body[1]), float(g_body[2])]
        self.latest_wb = [float(w_robot[0]), float(w_robot[1]), float(w_robot[2])]
        self.latest_quat = [float(w), float(x), float(y), float(z)]
        self.latest_heading_angle = float(heading_angle)
        self.latest_imu_raw_acc = [float(msg.imu_accel.x), float(msg.imu_accel.y), float(msg.imu_accel.z)]
        self.latest_imu_raw_gyro = [float(msg.imu_rate.x), float(msg.imu_rate.y), float(msg.imu_rate.z)]

        self.has_imu = True

    # =========================================================
    # Fixed-rate snapshot
    # =========================================================
    def record_snapshot(self):
        now_sec = self._now_sec()

        if self.time_start is None:
            self.time_start = now_sec

        elapsed = now_sec - self.time_start

        self.data['time'].append(elapsed)

        self.data['joint_pos'].append(self.latest_joint_pos.copy())
        self.data['joint_vel'].append(self.latest_joint_vel.copy())
        self.data['joint_effort'].append(self.latest_joint_effort.copy())

        self.data['cmd_pos'].append(self.latest_cmd_pos.copy())
        self.data['cmd_kp'].append(self.latest_cmd_kp.copy())
        self.data['cmd_kd'].append(self.latest_cmd_kd.copy())
        self.data['cmd_effort'].append(self.latest_cmd_effort.copy())

        self.data['joint_states_fb_torque'].append(self.latest_torque_fb.copy())

        self.data['torque_gravity_compensate'].append(self.latest_torque_gravity_compensate.copy())

        self.data['proj_g_x'].append(self.latest_proj_g[0])
        self.data['proj_g_y'].append(self.latest_proj_g[1])
        self.data['proj_g_z'].append(self.latest_proj_g[2])

        self.data['wb_x'].append(self.latest_wb[0])
        self.data['wb_y'].append(self.latest_wb[1])
        self.data['wb_z'].append(self.latest_wb[2])

        self.data['imu_raw_acc_x'].append(self.latest_imu_raw_acc[0])
        self.data['imu_raw_acc_y'].append(self.latest_imu_raw_acc[1])
        self.data['imu_raw_acc_z'].append(self.latest_imu_raw_acc[2])

        self.data['imu_raw_gyro_x'].append(self.latest_imu_raw_gyro[0])
        self.data['imu_raw_gyro_y'].append(self.latest_imu_raw_gyro[1])
        self.data['imu_raw_gyro_z'].append(self.latest_imu_raw_gyro[2])

        self.data['quat_w'].append(self.latest_quat[0])
        self.data['quat_x'].append(self.latest_quat[1])
        self.data['quat_y'].append(self.latest_quat[2])
        self.data['quat_z'].append(self.latest_quat[3])

        self.data['heading_angle'].append(self.latest_heading_angle)

        self.data['base_height_fb'].append(self.latest_base_height_fb)
        self.data['base_height_cmd'].append(self.latest_base_height_cmd)

        self.data['estimated_grf_paper_left_x'].append(self.latest_estimated_grf_paper_left[0])
        self.data['estimated_grf_paper_left_y'].append(self.latest_estimated_grf_paper_left[1])
        self.data['estimated_grf_paper_left_z'].append(self.latest_estimated_grf_paper_left[2])
        self.data['estimated_grf_paper_right_x'].append(self.latest_estimated_grf_paper_right[0])
        self.data['estimated_grf_paper_right_y'].append(self.latest_estimated_grf_paper_right[1])
        self.data['estimated_grf_paper_right_z'].append(self.latest_estimated_grf_paper_right[2])
        self.data['estimated_contact_state_left'].append(self.latest_estimated_contact_state[0])
        self.data['estimated_contact_state_right'].append(self.latest_estimated_contact_state[1])
        self.data['contact_score_left'].append(self.latest_contact_score[0])
        self.data['contact_score_right'].append(self.latest_contact_score[1])

    # =========================================================
    # Save CSV
    # =========================================================
    def save_to_csv(self):
        if 'time' not in self.data or len(self.data['time']) == 0:
            self.get_logger().warn("No data collected!")
            return

        try:
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)

                header = ['time']
                header.extend([f'{name}_pos' for name in self.joint_names])
                header.extend([f'{name}_vel' for name in self.joint_names])
                header.extend([f'{name}_effort' for name in self.joint_names])

                header.extend([f'{name}_cmd_pos' for name in self.joint_names])
                header.extend([f'{name}_cmd_kp' for name in self.joint_names])
                header.extend([f'{name}_cmd_kd' for name in self.joint_names])
                header.extend([f'{name}_cmd_effort' for name in self.joint_names])

                header.extend([f'{name}_fb_torque' for name in self.joint_names])

                header.extend([f'{name}_torque_gravity_compensate' for name in self.joint_names])

                header.extend([
                    'proj_g_x', 'proj_g_y', 'proj_g_z',
                    'wb_x', 'wb_y', 'wb_z',
                    'imu_raw_acc_x', 'imu_raw_acc_y', 'imu_raw_acc_z',
                    'imu_raw_gyro_x', 'imu_raw_gyro_y', 'imu_raw_gyro_z',
                    'quat_w', 'quat_x', 'quat_y', 'quat_z',
                    'heading_angle',
                    'base_height_fb', 'base_height_cmd',
                    'estimated_grf_paper_left_x', 'estimated_grf_paper_left_y', 'estimated_grf_paper_left_z',
                    'estimated_grf_paper_right_x', 'estimated_grf_paper_right_y', 'estimated_grf_paper_right_z',
                    'estimated_contact_state_left', 'estimated_contact_state_right',
                    'contact_score_left', 'contact_score_right'
                ])

                writer.writerow(header)

                num_samples = len(self.data['time'])
                for i in range(num_samples):
                    row = [self.data['time'][i]]

                    row.extend(self.data['joint_pos'][i])
                    row.extend(self.data['joint_vel'][i])
                    row.extend(self.data['joint_effort'][i])

                    row.extend(self.data['cmd_pos'][i])
                    row.extend(self.data['cmd_kp'][i])
                    row.extend(self.data['cmd_kd'][i])
                    row.extend(self.data['cmd_effort'][i])

                    row.extend(self.data['joint_states_fb_torque'][i])

                    row.extend(self.data['torque_gravity_compensate'][i])

                    row.append(self.data['proj_g_x'][i])
                    row.append(self.data['proj_g_y'][i])
                    row.append(self.data['proj_g_z'][i])

                    row.append(self.data['wb_x'][i])
                    row.append(self.data['wb_y'][i])
                    row.append(self.data['wb_z'][i])

                    row.append(self.data['imu_raw_acc_x'][i])
                    row.append(self.data['imu_raw_acc_y'][i])
                    row.append(self.data['imu_raw_acc_z'][i])
                    row.append(self.data['imu_raw_gyro_x'][i])
                    row.append(self.data['imu_raw_gyro_y'][i])
                    row.append(self.data['imu_raw_gyro_z'][i])

                    row.append(self.data['quat_w'][i])
                    row.append(self.data['quat_x'][i])
                    row.append(self.data['quat_y'][i])
                    row.append(self.data['quat_z'][i])

                    row.append(self.data['heading_angle'][i])

                    row.append(self.data['base_height_fb'][i])
                    row.append(self.data['base_height_cmd'][i])

                    row.append(self.data['estimated_grf_paper_left_x'][i])
                    row.append(self.data['estimated_grf_paper_left_y'][i])
                    row.append(self.data['estimated_grf_paper_left_z'][i])

                    row.append(self.data['estimated_grf_paper_right_x'][i])
                    row.append(self.data['estimated_grf_paper_right_y'][i])
                    row.append(self.data['estimated_grf_paper_right_z'][i])

                    row.append(self.data['estimated_contact_state_left'][i])
                    row.append(self.data['estimated_contact_state_right'][i])

                    row.append(self.data['contact_score_left'][i])
                    row.append(self.data['contact_score_right'][i])

                    writer.writerow(row)

            self.get_logger().info(f"Data saved to {self.csv_file}")

        except Exception as e:
            self.get_logger().error(f"Failed to save CSV: {e}")

    # =========================================================
    # Plot
    # =========================================================
    def plot_data(self):
        if 'time' not in self.data or len(self.data['time']) == 0:
            self.get_logger().warn("No data to plot!")
            return

        try:
            times = np.array(self.data['time'])

            joint_pos = np.array(self.data['joint_pos'])
            joint_vel = np.array(self.data['joint_vel'])
            joint_effort = np.array(self.data['joint_effort'])
            cmd_pos = np.array(self.data['cmd_pos'])
            cmd_effort = np.array(self.data['cmd_effort'])
            torque_fb = np.array(self.data['joint_states_fb_torque'])

            torque_gravity_compensate = np.array(self.data['torque_gravity_compensate'])

            plot_joint_order = [
                [
                    "R_hip_joint",
                    "R_hip2_joint",
                    "R_thigh_joint",
                    "R_calf_joint",
                    "R_toe_joint",
                ],
                [
                    "L_hip_joint",
                    "L_hip2_joint",
                    "L_thigh_joint",
                    "L_calf_joint",
                    "L_toe_joint",
                ],
            ]

            joint_name_to_index = {
                name: idx for idx, name in enumerate(self.joint_names)
            }

            # Figure 1: joint_pos + cmd_pos
            # fig1, axes1 = plt.subplots(2, 5, figsize=(20, 8))
            # fig1.suptitle(f'Joint Position vs Command Position - {self.timestamp}', fontsize=16)
            # axes1 = axes1.flatten()
            # for i in range(self.num_joints):
            #     ax = axes1[i]
            #     ax.plot(times, joint_pos[:, i], label='joint_pos')
            #     ax.plot(times, cmd_pos[:, i], label='cmd_pos')
            #     ax.set_title(self.joint_names[i])
            #     ax.set_xlabel('Time (s)')
            #     ax.set_ylabel('Position (rad)')
            #     ax.grid(True)
            #     ax.legend()
            # plt.tight_layout(rect=[0, 0, 1, 0.96])
            # plot_file_1 = self.csv_file.replace('.csv', '_joint_pos_cmd.png')
            # plt.savefig(plot_file_1, dpi=150)
            # self.get_logger().info(f"Joint position/command plot saved to {plot_file_1}")
            # plt.show()

            # Figure 1: joint_pos + cmd_pos
            # Plot order:
            #   Row 0: Right leg joints
            #   Row 1: Left leg joints
            fig1, axes1 = plt.subplots(2, 5, figsize=(20, 8))
            fig1.suptitle(f'Joint Position vs Command Position - {self.timestamp}', fontsize=16)

            for row in range(2):
                for col in range(5):
                    ax = axes1[row, col]
                    joint_name = plot_joint_order[row][col]

                    if joint_name not in joint_name_to_index:
                        ax.set_title(f'{joint_name} not found')
                        ax.axis('off')
                        continue

                    data_idx = joint_name_to_index[joint_name]

                    ax.plot(times, joint_pos[:, data_idx], label='joint_pos')
                    ax.plot(times, cmd_pos[:, data_idx], label='cmd_pos')
                    ax.set_title(joint_name)
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Position (rad)')
                    ax.grid(True)
                    ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_1 = self.csv_file.replace('.csv', '_joint_pos_cmd.png')
            plt.savefig(plot_file_1, dpi=150)
            self.get_logger().info(f"Joint position/command plot saved to {plot_file_1}")
            plt.show()

            # # Figure 2: joint_vel
            # fig2, axes2 = plt.subplots(2, 5, figsize=(20, 8))
            # fig2.suptitle(f'Joint Velocities - {self.timestamp}', fontsize=16)
            # axes2 = axes2.flatten()
            # for i in range(self.num_joints):
            #     ax = axes2[i]
            #     ax.plot(times, joint_vel[:, i], label='joint_vel')
            #     ax.set_title(self.joint_names[i])
            #     ax.set_xlabel('Time (s)')
            #     ax.set_ylabel('Velocity (rad/s)')
            #     ax.grid(True)
            #     ax.legend()
            # plt.tight_layout(rect=[0, 0, 1, 0.96])
            # plot_file_2 = self.csv_file.replace('.csv', '_joint_vel.png')
            # plt.savefig(plot_file_2, dpi=150)
            # self.get_logger().info(f"Joint velocity plot saved to {plot_file_2}")
            # plt.show()

            # Figure 2: joint_vel
            fig2, axes2 = plt.subplots(2, 5, figsize=(20, 8))
            fig2.suptitle(f'Joint Velocities - {self.timestamp}', fontsize=16)

            for row in range(2):
                for col in range(5):
                    ax = axes2[row, col]
                    joint_name = plot_joint_order[row][col]

                    if joint_name not in joint_name_to_index:
                        ax.set_title(f'{joint_name} not found')
                        ax.axis('off')
                        continue

                    data_idx = joint_name_to_index[joint_name]

                    ax.plot(times, joint_vel[:, data_idx], label='joint_vel')
                    ax.set_title(joint_name)
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Velocity (rad/s)')
                    ax.grid(True)
                    ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_2 = self.csv_file.replace('.csv', '_joint_vel.png')
            plt.savefig(plot_file_2, dpi=150)
            self.get_logger().info(f"Joint velocity plot saved to {plot_file_2}")
            plt.show()

            # # Figure 3: torque
            # fig3, axes3 = plt.subplots(2, 5, figsize=(20, 8))
            # fig3.suptitle(f'Joint Torque - {self.timestamp}', fontsize=16)
            # axes3 = axes3.flatten()
            # for i in range(self.num_joints):
            #     ax = axes3[i]
            #     ax.plot(times, joint_effort[:, i], label='joint_states.effort')
            #     ax.plot(times, cmd_effort[:, i], label='joint_cmds.effort')
            #     ax.plot(times, torque_fb[:, i], label='joint_states_fb_torque')
            #     ax.set_title(self.joint_names[i])
            #     ax.set_xlabel('Time (s)')
            #     ax.set_ylabel('Torque (Nm)')
            #     ax.grid(True)
            #     ax.legend()
            # plt.tight_layout(rect=[0, 0, 1, 0.96])
            # plot_file_3 = self.csv_file.replace('.csv', '_joint_torque.png')
            # plt.savefig(plot_file_3, dpi=150)
            # self.get_logger().info(f"Joint torque plot saved to {plot_file_3}")
            # plt.show()

            # Figure 3: torque
            fig3, axes3 = plt.subplots(2, 5, figsize=(20, 8))
            fig3.suptitle(f'Joint Torque - {self.timestamp}', fontsize=16)

            for row in range(2):
                for col in range(5):
                    ax = axes3[row, col]
                    joint_name = plot_joint_order[row][col]

                    if joint_name not in joint_name_to_index:
                        ax.set_title(f'{joint_name} not found')
                        ax.axis('off')
                        continue

                    data_idx = joint_name_to_index[joint_name]

                    ax.plot(times, joint_effort[:, data_idx], label='joint_states.effort')
                    ax.plot(times, torque_fb[:, data_idx], label='joint_states_fb_torque')
                    ax.set_title(joint_name)
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Torque (Nm)')
                    ax.grid(True)
                    ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_3 = self.csv_file.replace('.csv', '_joint_torque.png')
            plt.savefig(plot_file_3, dpi=150)
            self.get_logger().info(f"Joint torque plot saved to {plot_file_3}")
            plt.show()

            # Figure 4: projected gravity
            fig4, axes4 = plt.subplots(1, 3, figsize=(15, 4))
            fig4.suptitle(f'Projected Gravity - {self.timestamp}', fontsize=16)
            proj_g_x = np.array(self.data['proj_g_x'])
            proj_g_y = np.array(self.data['proj_g_y'])
            proj_g_z = np.array(self.data['proj_g_z'])

            axes4[0].plot(times, proj_g_x)
            axes4[0].set_title('Projected Gravity X')
            axes4[0].set_xlabel('Time (s)')
            axes4[0].set_ylabel('Value')
            axes4[0].grid(True)

            axes4[1].plot(times, proj_g_y)
            axes4[1].set_title('Projected Gravity Y')
            axes4[1].set_xlabel('Time (s)')
            axes4[1].set_ylabel('Value')
            axes4[1].grid(True)

            axes4[2].plot(times, proj_g_z)
            axes4[2].set_title('Projected Gravity Z')
            axes4[2].set_xlabel('Time (s)')
            axes4[2].set_ylabel('Value')
            axes4[2].grid(True)

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_4 = self.csv_file.replace('.csv', '_proj_gravity.png')
            plt.savefig(plot_file_4, dpi=150)
            self.get_logger().info(f"Projected gravity plot saved to {plot_file_4}")
            plt.show()

            # Figure 5: angular velocities and heading
            fig5, axes5 = plt.subplots(2, 2, figsize=(12, 8))
            fig5.suptitle(f'Base Angular Velocity and Heading Angle - {self.timestamp}', fontsize=16)
            axes5 = axes5.flatten()

            wb_x = np.array(self.data['wb_x'])
            wb_y = np.array(self.data['wb_y'])
            wb_z = np.array(self.data['wb_z'])
            heading_angle = np.array(self.data['heading_angle'])

            axes5[0].plot(times, wb_x)
            axes5[0].set_title('Angular Velocity X')
            axes5[0].set_xlabel('Time (s)')
            axes5[0].set_ylabel('Angular Velocity (rad/s)')
            axes5[0].grid(True)

            axes5[1].plot(times, wb_y)
            axes5[1].set_title('Angular Velocity Y')
            axes5[1].set_xlabel('Time (s)')
            axes5[1].set_ylabel('Angular Velocity (rad/s)')
            axes5[1].grid(True)

            axes5[2].plot(times, wb_z)
            axes5[2].set_title('Angular Velocity Z')
            axes5[2].set_xlabel('Time (s)')
            axes5[2].set_ylabel('Angular Velocity (rad/s)')
            axes5[2].grid(True)

            axes5[3].plot(times, heading_angle)
            axes5[3].set_title('Heading Angle')
            axes5[3].set_xlabel('Time (s)')
            axes5[3].set_ylabel('Heading Angle (rad)')
            axes5[3].grid(True)

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_5 = self.csv_file.replace('.csv', '_ang_vel_heading.png')
            plt.savefig(plot_file_5, dpi=150)
            self.get_logger().info(f"Angular velocity and heading angle plot saved to {plot_file_5}")
            plt.show()

            # Figure 6: base height feedback + command
            fig6 = plt.figure(figsize=(10, 5))
            plt.plot(times, np.array(self.data['base_height_fb']), label='base_height_fb')
            plt.plot(times, np.array(self.data['base_height_cmd']), label='base_height_cmd')
            plt.title(f'Base Height Feedback vs Command - {self.timestamp}')
            plt.xlabel('Time (s)')
            plt.ylabel('Height (m)')
            plt.grid(True)
            plt.legend()

            plot_file_6 = self.csv_file.replace('.csv', '_base_height.png')
            plt.savefig(plot_file_6, dpi=150)
            self.get_logger().info(f"Base height plot saved to {plot_file_6}")
            plt.show()

            # # Figure 7: torque gravity compensate
            # fig7, axes7 = plt.subplots(2, 5, figsize=(20, 8))
            # fig7.suptitle(f'Torque gravity compensate - {self.timestamp}', fontsize=16)
            # axes7 = axes7.flatten()
            # for i in range(self.num_joints):
            #     ax = axes7[i]
            #     ax.plot(times, torque_gravity_compensate[:, i], label='torque gravity compensate')
            #     ax.set_title(self.joint_names[i])
            #     ax.set_xlabel('Time (s)')
            #     ax.set_ylabel('Torque (Nm)')
            #     ax.grid(True)
            #     ax.legend()
            # plt.tight_layout(rect=[0, 0, 1, 0.96])
            # plot_file_7 = self.csv_file.replace('.csv', '_gravity_compensate.png')
            # plt.savefig(plot_file_7, dpi=150)
            # self.get_logger().info(f"Joint velocity plot saved to {plot_file_7}")
            # plt.show()

            # Figure 7: torque gravity compensate
            fig7, axes7 = plt.subplots(2, 5, figsize=(20, 8))
            fig7.suptitle(f'Torque Gravity Compensate - {self.timestamp}', fontsize=16)

            for row in range(2):
                for col in range(5):
                    ax = axes7[row, col]
                    joint_name = plot_joint_order[row][col]

                    if joint_name not in joint_name_to_index:
                        ax.set_title(f'{joint_name} not found')
                        ax.axis('off')
                        continue

                    data_idx = joint_name_to_index[joint_name]

                    ax.plot(
                        times,
                        torque_gravity_compensate[:, data_idx],
                        label='torque gravity compensate'
                    )
                    ax.set_title(joint_name)
                    ax.set_xlabel('Time (s)')
                    ax.set_ylabel('Torque (Nm)')
                    ax.grid(True)
                    ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_7 = self.csv_file.replace('.csv', '_gravity_compensate.png')
            plt.savefig(plot_file_7, dpi=150)
            self.get_logger().info(f"Torque gravity compensate plot saved to {plot_file_7}")
            plt.show()

            # Figure 8: estimated GRF paper + contact score/state, 2x2
            fig8, axes8 = plt.subplots(2, 2, figsize=(18, 10))
            fig8.suptitle(f'Estimated GRF Paper and Contact State - {self.timestamp}', fontsize=16)

            grf_left_x = np.array(self.data['estimated_grf_paper_left_x'])
            grf_left_y = np.array(self.data['estimated_grf_paper_left_y'])
            grf_left_z = np.array(self.data['estimated_grf_paper_left_z'])

            grf_right_x = np.array(self.data['estimated_grf_paper_right_x'])
            grf_right_y = np.array(self.data['estimated_grf_paper_right_y'])
            grf_right_z = np.array(self.data['estimated_grf_paper_right_z'])

            contact_state_left = np.array(self.data['estimated_contact_state_left'])
            contact_state_right = np.array(self.data['estimated_contact_state_right'])

            contact_score_left = np.array(self.data['contact_score_left'])
            contact_score_right = np.array(self.data['contact_score_right'])

            # Scale contact state lên ngang hàng với contact score để dễ nhìn
            left_score_max = np.nanmax(np.abs(contact_score_left)) if contact_score_left.size > 0 else 1.0
            right_score_max = np.nanmax(np.abs(contact_score_right)) if contact_score_right.size > 0 else 1.0

            if left_score_max < 1e-6:
                left_score_max = 1.0

            if right_score_max < 1e-6:
                right_score_max = 1.0

            contact_state_left_scaled = contact_state_left * left_score_max
            contact_state_right_scaled = contact_state_right * right_score_max

            # -----------------------------
            # Top-left: GRF Left
            # -----------------------------
            ax = axes8[0, 0]
            ax.plot(times, grf_left_x, label='GRF left Fx')
            ax.plot(times, grf_left_y, label='GRF left Fy')
            ax.plot(times, grf_left_z, label='GRF left Fz')
            ax.set_title('Estimated GRF Paper - Left Foot')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('GRF (N)')
            ax.grid(True)
            ax.legend()

            # -----------------------------
            # Top-right: GRF Right
            # -----------------------------
            ax = axes8[0, 1]
            ax.plot(times, grf_right_x, label='GRF right Fx')
            ax.plot(times, grf_right_y, label='GRF right Fy')
            ax.plot(times, grf_right_z, label='GRF right Fz')
            ax.set_title('Estimated GRF Paper - Right Foot')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('GRF (N)')
            ax.grid(True)
            ax.legend()

            # -----------------------------
            # Bottom-left: Contact Left
            # -----------------------------
            ax = axes8[1, 0]
            ax.plot(times, contact_score_left, label='left contact score')
            ax.step(
                times,
                contact_state_left_scaled,
                where='post',
                label=f'left contact state scaled x{left_score_max:.1f}'
            )
            ax.set_title('Contact Score and State - Left Foot')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Score / Scaled Contact State')
            ax.grid(True)
            ax.legend()

            # -----------------------------
            # Bottom-right: Contact Right
            # -----------------------------
            ax = axes8[1, 1]
            ax.plot(times, contact_score_right, label='right contact score')
            ax.step(
                times,
                contact_state_right_scaled,
                where='post',
                label=f'right contact state scaled x{right_score_max:.1f}'
            )
            ax.set_title('Contact Score and State - Right Foot')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Score / Scaled Contact State')
            ax.grid(True)
            ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.96])
            plot_file_8 = self.csv_file.replace('.csv', '_estimated_grf_paper_contact_2x2.png')
            plt.savefig(plot_file_8, dpi=150)
            self.get_logger().info(f"Estimated GRF paper/contact 2x2 plot saved to {plot_file_8}")
            plt.show()

            # Figure 9: selected joint position feedback + opposite contact state
            fig9, axes9 = plt.subplots(1, 2, figsize=(16, 5))
            fig9.suptitle(f'Selected Joint Position Feedback vs Contact State - {self.timestamp}', fontsize=16)

            joint_pos_feedback = np.array(self.data['joint_pos'])

            contact_state_left = np.array(self.data['estimated_contact_state_left'])
            contact_state_right = np.array(self.data['estimated_contact_state_right'])

            # Joint index 3 and 7
            joint_idx_right = 3
            joint_idx_left = 7

            joint_pos_idx3 = joint_pos_feedback[:, joint_idx_right]
            joint_pos_idx7 = joint_pos_feedback[:, joint_idx_left]

            # Scale contact state to be visible together with joint position.
            # contact_scaled = contact_state * max(abs(joint_pos))
            scale_idx3 = np.nanmax(np.abs(joint_pos_idx3)) if joint_pos_idx3.size > 0 else 1.0
            scale_idx7 = np.nanmax(np.abs(joint_pos_idx7)) if joint_pos_idx7.size > 0 else 1.0

            if scale_idx3 < 1e-6:
                scale_idx3 = 1.0

            if scale_idx7 < 1e-6:
                scale_idx7 = 1.0

            contact_state_right_scaled = contact_state_right * scale_idx3
            contact_state_left_scaled = contact_state_left * scale_idx7

            # -----------------------------
            # Left plot: joint pos index 3 + right contact state
            # -----------------------------
            ax = axes9[0]
            ax.plot(
                times,
                joint_pos_idx3,
                label=f'{self.joint_names[joint_idx_right]} pos feedback'
            )
            ax.step(
                times,
                contact_state_right_scaled,
                where='post',
                label=f'right contact state scaled x{scale_idx3:.3f}'
            )
            ax.set_title(f'Joint Pos Index {joint_idx_right} + Right Contact State')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Joint Position (rad) / Scaled Contact')
            ax.grid(True)
            ax.legend()

            # -----------------------------
            # Right plot: joint pos index 7 + left contact state
            # -----------------------------
            ax = axes9[1]
            ax.plot(
                times,
                joint_pos_idx7,
                label=f'{self.joint_names[joint_idx_left]} pos feedback'
            )
            ax.step(
                times,
                contact_state_left_scaled,
                where='post',
                label=f'left contact state scaled x{scale_idx7:.3f}'
            )
            ax.set_title(f'Joint Pos Index {joint_idx_left} + Left Contact State')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Joint Position (rad) / Scaled Contact')
            ax.grid(True)
            ax.legend()

            plt.tight_layout(rect=[0, 0, 1, 0.94])
            plot_file_9 = self.csv_file.replace('.csv', '_joint_pos_contact_state.png')
            plt.savefig(plot_file_9, dpi=150)
            self.get_logger().info(f"Joint position/contact state plot saved to {plot_file_9}")
            plt.show()

        except Exception as e:
            self.get_logger().error(f"Failed to plot data: {e}")


def main(args=None):
    rclpy.init(args=args)
    recorder = DataRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        print("\nStopping data recorder...")
        recorder.save_to_csv()
        recorder.plot_data()
        print("Data recorder stopped.")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()