
# Copyright (c) 2022 Birdiebot R&D Department
# Shanghai University Of Engineering Science. All Rights Reserved
# License: GNU General Public License v3.0.
# See LICENSE file in root directory.
# 
# Author: ligcox ligcox@birdiebot.top
# Date: 2022-09-01 21:26:46
# FilePath: /bubble/src/bubble_core/bubble_protocol/bubble_protocol/dispatch.py
# LastEditors: ligcox ligcox@birdiebot.top
# LastEditTime: 2022-09-01 22:08:16

'''
Robot communication dispatch layer, 
this module is the bridge between DDS and serial port hardware. 
Through the module, 
a BCP frame can be initialized to receive the data transmitted by DDS 
and send the data to MCU. 
At the same time, the data received by the onboard computer will also be 
transmitted to the DDS through this module 
to be received by other subscribers.
'''
import time

from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Int8

from bubble_protocol.hardware import RobotSerial
from bubble_protocol.robot_status import RobotStatus
from rmctrl_msgs.msg import Chassis, Gimbal, Shooter


class RobotAPI(Node):
    """Generate a new BCP core.

    Attributes
    ----------
    robot_status: `RobotStatus`
        The robot current status.
    robot_serial: `Serial`
        Onboard serial port object.

    Examples
    --------
    >>> from bubble_protocol.dispatch import RobotAPI
    >>> core = RobotAPI()
    """

    def __init__(self, name="standard") -> None:
        """
        Generate a new BCP core.
        """
        super().__init__("BCP_Core")

        # Get params from ROS2 launch system
        self.declare_parameter('robot_type', 'None')
        self.declare_parameter('serial_port', 'None')
        name = self.get_parameter(
            'robot_type').get_parameter_value().string_value
        serial_port = self.get_parameter(
            'serial_port').get_parameter_value().string_value
        if name != 'None':
            self.get_logger().info(f'Robot {name} has been initialized...')
        else:
            self.get_logger().error(
                f'No robot has been initialized, please check the parameter Settings!')
            self.get_logger().error(
                f'Robot type:{name}')
            raise ValueError(
                'No robot has been initialized, please check the parameter Settings!')

        # Init robot core hardware
        serial_opened = False
        while not serial_opened:
            try:
                self.robot_serial = RobotSerial(name, port=serial_port)
                serial_opened = True
            except Exception as e:
                self.get_logger().error(
                    f'Open serial port error, try to reopen port:{serial_port}, info: {e}')
                time.sleep(3)
        self.robot_status = RobotStatus(self.robot_serial.status, self)
        self.robot_serial.realtime_pub = self.robot_status.realtime_callback
        self.robot_serial.serial_done = True

        # init core api
        self.api_init()
        # init expanded api
        self.init_robot(name)
        # Init robot tx/rx/heartbeat timer
        self.uart_timer = self.create_timer(0.01, self.robot_serial.process)
        self.uartrx_timer = self.create_timer(
            0.01, self.robot_serial.rx_function)
        self.heartbeat_timer = self.create_timer(0.5, self.heartbeat)

    def api_init(self) -> None:
        '''General information API of robot definition.
        '''
        # heartbeat data initalized
        self.heartbeat_time = 0

        # subscriber api
        self.mode_ctrl_sub = self.create_subscription(
            Int8, '/core/mode_api', self.mode_ctrl_callback, 10)
        self.gimbal_sub = self.create_subscription(
            Gimbal, '/core/gimbal_api', self.gimbal_callback, 1)
        self.barrel_sub = self.create_subscription(
            Shooter, '/core/shooter_api', self.barrel_callback, 1)

    def heartbeat(self):
        '''heartbeat function, send heartbeat frames periodically.
        '''
        self.robot_serial.send_data("heartbeat", [self.heartbeat_time])
        # self.get_logger().info("heartbeat_time: {},time:{}".format(self.heartbeat_time,time.time()))
        self.heartbeat_time = 1 if self.heartbeat_time == 0 else 0

    def mode_ctrl_callback(self, msg: Int8) -> None:
        '''mode control function, send mode infomation to MCU.

        Parameters
        ------------
        msg: `Int8`
            A mode message is received
        '''
        # self.get_logger().info("recived data mode value: {}".format(msg.data))
        self.robot_serial.send_data("mode", [msg.data])

    def gimbal_callback(self, msg: Gimbal) -> None:
        """gimbal function, send gimbal infomation to MCU.

        Parameters
        ----------
        msg: `Gimbal`
            A mode message is received
        """
        # self.get_logger().info("recived data gimbal, yaw: {}, pitch: {}, roll: {}".format(
        #     msg.yaw, msg.pitch, msg.roll))
        self.robot_serial.send_data(
            "gimbal", [msg.mode, msg.yaw, msg.pitch, msg.roll])

    def barrel_callback(self, msg: Shooter) -> None:
        """Shooter function, send enable shooter infomation to MCU.

        Parameters
        ----------
        msg: `Shooter`
            A mode message is received
        """
        # self.get_logger().info("recived data barrel change, barrel: {}".format(msg.data))
        self.robot_serial.send_data(
            "barrel", [msg.is_shoot, msg.bullet_vel, msg.remain_bullet])

    def init_robot(self, name):
        """Expanded api definition, API required by the special robot is initialized.

        Parameters
        ----------
        name: `str`
            robot name, name option reference 
            `The Chinese-English comparison table <https://birdiebot.github.io/bubble_documentation/guide/%E6%9C%AF%E8%AF%AD%E4%B8%AD%E8%8B%B1%E6%96%87%E5%AF%B9%E7%85%A7%E8%A1%A8.html>`__ .
        """

        if name == "sentry_up":
            self.chassis_sub = self.create_subscription(
                Chassis, '/core/chassis_api', self.ex_chassis_callback, 10)
        elif name == "sentry_down":
            self.chassis_sub = self.create_subscription(
                Chassis, '/core/chassis_api', self.ex_chassis_callback, 10)
        elif name == "infantry":
            pass
        elif name == "engineer":
            pass
        elif name == "hero":
            pass
        elif name == "air":
            pass
        elif name == "radar":
            pass
        elif name == "gather":
            pass
        elif name == "standard":
            self.chassis_sub = self.create_subscription(
                Chassis, '/core/chassis_api', self.ex_chassis_callback, 10)
            self.odom_sub = self.create_subscription(
                PoseWithCovariance, "odom", self.ex_odom_callback, 1)

    def ex_chassis_callback(self, msg: Chassis) -> None:
        """Chassis function, send chassis infomation to MCU.

        Parameters
        ----------
        msg: `Chassis`
            A chassis message is received
        """
        self.robot_serial.send_data("chassis_ctrl", [msg.chassis_target_linear_x, msg.chassis_target_linear_y,
                                    msg.chassis_target_linear_z, msg.chassis_target_angular_x, msg.chassis_target_angular_y, msg.chassis_target_angular_z])

    def ex_odom_callback(self, msg: PoseWithCovariance):
        """Odoo function, send chassis infomation to MCU.

        Parameters
        ----------
        msg: `PoseWithCovariance`
            A chassis message is received
        """
        odom_list = []
        odom_list.append(msg.pose.position.x)
        odom_list.append(msg.pose.position.y)
        odom_list.append(msg.pose.position.z)
        odom_list.append(msg.pose.orientation.x)
        odom_list.append(msg.pose.orientation.y)
        odom_list.append(msg.pose.orientation.z)
        odom_list.append(msg.pose.orientation.w)
        self.robot_serial.send_data("odom", odom_list)
