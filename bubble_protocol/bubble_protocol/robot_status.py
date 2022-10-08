
# Copyright (c) 2022 Birdiebot R&D Department
# Shanghai University Of Engineering Science. All Rights Reserved
# License: GNU General Public License v3.0.
# See LICENSE file in root directory.
# 
# Author: ligcox ligcox@birdiebot.top
# Date: 2022-09-01 22:14:57
# FilePath: /bubble/src/bubble_core/bubble_protocol/bubble_protocol/robot_status.py
# LastEditors: ligcox ligcox@birdiebot.top
# LastEditTime: 2022-09-01 22:20:49


# Copyright (c) 2022 Birdiebot R&D Department
# Shanghai University Of Engineering Science. All Rights Reserved
# License: GNU General Public License v3.0.
# See LICENSE file in root directory.
#
# Author: ligcox ligcox@birdiebot.top
# Date: 2022-09-01 22:12:34
# FilePath: /bubble/src/bubble_core/bubble_protocol/bubble_protocol/robot_status.py
# LastEditors: ligcox ligcox@birdiebot.top
# LastEditTime: 2022-09-01 22:14:23

'''
Robot communication status layer,
and this module may be refactored in the future.
This module will publish the robot status data sent by MCU
to onboard to DDS for other subscriber to receice.
'''

from rclpy.node import Node

from std_msgs.msg import Int8
import game_msgs.msg
import rmctrl_msgs
import rmctrl_msgs.msg

from bubble_protocol.protocol import *


class RobotStatus():
    """Send robot status information.

    Attributes
    ----------
    node: `Node`
        Node of maintain robot status information.
    status: `STATUS`
        Robot current status data.
    """

    def __init__(self, status: dict, node: Node) -> None:
        self.node = node
        self.status = status
        self.realtime_callback = REALTIME_CALLBACK
        self.status_init()

        self.non_realtime_timer = self.node.create_timer(
            0.1, self.non_realtime_status)

    def status_init(self) -> None:
        ''' The function defines publishes required for the robot status.
        '''
        def gimbal_callback():
            gimbal_msg = rmctrl_msgs.msg.Gimbal()
            gimbal_msg.header.stamp = self.node.get_clock().now().to_msg()
            gimbal_msg.yaw = float(
                self.status["gimbal"]["gimbal_yaw"][IDX_VAL])
            gimbal_msg.pitch = float(
                self.status["gimbal"]["gimbal_pitch"][IDX_VAL])
            gimbal_msg.roll = float(
                self.status["gimbal"]["gimbal_roll"][IDX_VAL])
            self.gimbal_pub.publish(gimbal_msg)

        def shooter_callback():
            bullet_msg = rmctrl_msgs.msg.Shooter()
            bullet_msg.is_shoot = bool(
                self.status["barrel"]["is_shoot"][IDX_VAL])
            bullet_msg.bullet_vel = int(
                self.status["barrel"]["bullet_vel"][IDX_VAL])
            bullet_msg.remain_bullet = int(
                self.status["barrel"]["remain_bullet"][IDX_VAL])
            self.barrel_pub.publish(bullet_msg)

        def chassis_callback():
            chassis_msg = rmctrl_msgs.msg.Chassis()
            chassis_msg.chassis_target_linear_x = float(
                self.status["chassis_ctrl"]["chassis_target_linear_x"][IDX_VAL])
            chassis_msg.chassis_target_linear_y = float(
                self.status["chassis_ctrl"]["chassis_target_linear_y"][IDX_VAL])
            chassis_msg.chassis_target_linear_z = float(
                self.status["chassis_ctrl"]["chassis_target_linear_z"][IDX_VAL])
            chassis_msg.chassis_target_angular_x = float(
                self.status["chassis_ctrl"]["chassis_target_angular_x"][IDX_VAL])
            chassis_msg.chassis_target_angular_y = float(
                self.status["chassis_ctrl"]["chassis_target_angular_y"][IDX_VAL])
            chassis_msg.chassis_target_angular_z = float(
                self.status["chassis_ctrl"]["chassis_target_angular_z"][IDX_VAL])
            self.chassis_pub.publish(chassis_msg)

        # real-time publisher api
        self.gimbal_pub = self.node.create_publisher(
            rmctrl_msgs.msg.Gimbal, '/status/gimbal', 10)
        self.barrel_pub = self.node.create_publisher(
            rmctrl_msgs.msg.Shooter, '/status/barrel', 10)
        self.chassis_pub = self.node.create_publisher(
            rmctrl_msgs.msg.Chassis, '/status/chassis', 10)
        self.realtime_callback["gimbal"] = gimbal_callback
        self.realtime_callback["barrel"] = shooter_callback
        self.realtime_callback["chassis_ctrl"] = chassis_callback

        # non-realtime publisher api
        self.manifold_ctrl_pub = self.node.create_publisher(
            Int8, '/status/manifold_ctrl', 10)
        self.gameStatus_pub = self.node.create_publisher(
            game_msgs.msg.GameStatus, '/status/game', 10)
        self.zone_pub = self.node.create_publisher(
            game_msgs.msg.Zone, '/status/zone', 10)
        self.hp_pub = self.node.create_publisher(
            game_msgs.msg.RobotHP, '/status/robotHP', 10)
        self.game_mode_pub = self.node.create_publisher(
            Int8, '/status/game_mode', 10)

    def non_realtime_status(self) -> None:
        '''The function defines data publish rules of 
        robot's non-realtime status. It will executed perodically by timer. 
        The period is usually 1 second.
        '''

        game_mode_msg = Int8()
        game_mode_msg.data = int(
            self.status["game_mode"]["game_mode"][IDX_VAL])
        self.game_mode_pub.publish(game_mode_msg)

        manifold_ctrl_msg = Int8()
        manifold_ctrl_msg.data = int(
            self.status["manifold_ctrl"]["mode_ctrl"][IDX_VAL])
        self.manifold_ctrl_pub.publish(manifold_ctrl_msg)

        gameStatus_msg = game_msgs.msg.GameStatus()
        gameStatus_msg.game_type = int(
            self.status["game_status"]["game_type"][IDX_VAL])
        gameStatus_msg.game_progress = int(
            self.status["game_status"]["game_progress"][IDX_VAL])
        gameStatus_msg.stage_remain_time = self.status["game_status"]["stage_remain_time"][IDX_VAL]
        self.gameStatus_pub.publish(gameStatus_msg)

        zone_msg = game_msgs.msg.Zone()
        zone_msg.f1_zone_status = self.status["ICRA_buff_debuff_zone"]["F1_zone_status"][IDX_VAL]
        zone_msg.f1_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F1_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f2_zone_status = self.status["ICRA_buff_debuff_zone"]["F2_zone_status"][IDX_VAL]
        zone_msg.f2_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F2_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f3_zone_status = self.status["ICRA_buff_debuff_zone"]["F3_zone_status"][IDX_VAL]
        zone_msg.f3_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F3_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f4_zone_status = self.status["ICRA_buff_debuff_zone"]["F4_zone_status"][IDX_VAL]
        zone_msg.f4_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F4_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f5_zone_status = self.status["ICRA_buff_debuff_zone"]["F5_zone_status"][IDX_VAL]
        zone_msg.f5_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F5_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.f6_zone_status = self.status["ICRA_buff_debuff_zone"]["F6_zone_status"][IDX_VAL]
        zone_msg.f6_zone_buff_debuff_status = self.status[
            "ICRA_buff_debuff_zone"]["F6_zone_buff_debuff_status"][IDX_VAL]
        zone_msg.red1_bullet_left = self.status["ICRA_buff_debuff_zone"]["red1_bullet_left"][IDX_VAL]
        zone_msg.red2_bullet_left = self.status["ICRA_buff_debuff_zone"]["red2_bullet_left"][IDX_VAL]
        zone_msg.blue1_bullet_left = self.status["ICRA_buff_debuff_zone"]["blue1_bullet_left"][IDX_VAL]
        zone_msg.blue2_bullet_left = self.status["ICRA_buff_debuff_zone"]["blue2_bullet_left"][IDX_VAL]
        self.zone_pub.publish(zone_msg)

        robotHP_msg = game_msgs.msg.RobotHP()
        robotHP_msg.red_1_robot_hp = int(
            self.status["robot_HP"]["red_1_robot_HP"][IDX_VAL])
        robotHP_msg.red_2_robot_hp = int(
            self.status["robot_HP"]["red_2_robot_HP"][IDX_VAL])
        robotHP_msg.red_3_robot_hp = int(
            self.status["robot_HP"]["red_3_robot_HP"][IDX_VAL])
        robotHP_msg.red_4_robot_hp = int(
            self.status["robot_HP"]["red_4_robot_HP"][IDX_VAL])
        robotHP_msg.red_5_robot_hp = int(
            self.status["robot_HP"]["red_5_robot_HP"][IDX_VAL])
        robotHP_msg.red_7_robot_hp = int(
            self.status["robot_HP"]["red_7_robot_HP"][IDX_VAL])
        robotHP_msg.red_outpost_hp = int(
            self.status["robot_HP"]["red_outpost_HP"][IDX_VAL])
        robotHP_msg.red_base_hp = int(
            self.status["robot_HP"]["red_base_HP"][IDX_VAL])
        robotHP_msg.blue_1_robot_hp = int(
            self.status["robot_HP"]["blue_1_robot_HP"][IDX_VAL])
        robotHP_msg.blue_2_robot_hp = int(
            self.status["robot_HP"]["blue_2_robot_HP"][IDX_VAL])
        robotHP_msg.blue_3_robot_hp = int(
            self.status["robot_HP"]["blue_3_robot_HP"][IDX_VAL])
        robotHP_msg.blue_4_robot_hp = int(
            self.status["robot_HP"]["blue_4_robot_HP"][IDX_VAL])
        robotHP_msg.blue_5_robot_hp = int(
            self.status["robot_HP"]["blue_5_robot_HP"][IDX_VAL])
        robotHP_msg.blue_7_robot_hp = int(
            self.status["robot_HP"]["blue_7_robot_HP"][IDX_VAL])
        robotHP_msg.blue_outpost_hp = int(
            self.status["robot_HP"]["blue_outpost_HP"][IDX_VAL])
        robotHP_msg.blue_base_hp = int(
            self.status["robot_HP"]["blue_base_HP"][IDX_VAL])
        self.hp_pub.publish(robotHP_msg)
