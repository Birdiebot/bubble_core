'''
Copyright (c) 2022 Birdiebot R&D Department
Shanghai University Of Engineering Science. All Rights Reserved

License: GNU General Public License v3.0.
See LICENSE file in root directory.
Author: Ligcox
Date: 2022-05-10 18:59:22
FilePath: /bubble_bringup/home/nvidia/Desktop/bubble/src/bubble_core/bubble_protocol/bubble_protocol/bcp_core.py
LastEditors: Ligcox
LastEditTime: 2022-05-12 04:44:57
E-mail: robomaster@birdiebot.top
'''

from dispatch import RobotAPI
from protocol import RefereeRobotID


def main(args=None):
    bcp_core = RobotAPI(name='radar', serial_port='ttyTHS0')

    # get client delivers information
    data = bcp_core.get_status()
    target_position_x = data["robot_command"]["target_position_x"]
    target_position_y = data["robot_command"]["target_position_y"]
    target_position_z = data["robot_command"]["target_position_z"]
    commd_keyboard = data["robot_command"]["commd_keyboard"]
    target_robot_ID = data["robot_command"]["target_robot_ID"]
    '''
        DO TOUR CODE DISPLAY TO CLIENT UI
    '''

    # send information to the client
    # robot target id your will send, assuming red air
    '''
        DO TOUR CODE GET ROBOT POSITION
    '''
    target_robot_ID = RefereeRobotID["red_air"]
    # robot target x pose your will send
    target_position_x = 1.2
    # robot target y pose your will send
    target_position_y = 1.3
    bcp_core.send_radar_data(target_robot_ID, target_position_x, target_position_y)


if __name__ == '__main__':
    main()
