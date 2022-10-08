'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-30 01:08:04
FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_core/bubble_protocol/bubble_protocol/dispatch.py
LastEditors: Ligcox
LastEditTime: 2022-07-02 19:57:11
E-mail: robomaster@birdiebot.top
'''

import threading
import time

from hardware import RobotSerial


class RobotAPI():
    def __init__(self, name="standard", serial_port='ttyTHS0'):
        if name != 'None':
            self.get_logger().info(f'Robot {name} has been initialized...')
        else:
            print(
                f'No robot has been initialized, please check the parameter Settings!')
            print(
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
        self.robot_serial.serial_done = True

        # init core api
        self.api_init()
        # init expanded api
        self.init_robot(name)

        # Thread operation
        self.heartbeat_thread = threading.Thread(
            target=self.heartbeat, name="heartbeat")
        self.tx_thread = threading.Thread(
            target=self.robot_serial.process, name="uart_thread")

        self.heartbeat_thread.start()
        self.tx_thread.start()

    # General information definition

    def api_init(self):
        # TODO
        pass

    def heartbeat(self):
        while True:
            self.robot_serial.send_data("heartbeat", [self.heartbeat_time])
            self.heartbeat_time = 1 if self.heartbeat_time == 0 else 0
            time.sleep(0.5)

    def send_radar_data(self, target_robot_ID, target_position_x, target_position_y):
        self.robot_serial.send_data("client_map_command_command", [
                                    target_robot_ID, target_position_x, target_position_y])

    def get_status(self):
        return self.robot_serial.status

    # Expanded api definition
    def init_robot(self, name):
        if name == "sentry_up":
            pass
        elif name == "sentry_down":
            pass
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
            pass