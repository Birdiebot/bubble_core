
# Copyright (c) 2022 Birdiebot R&D Department
# Shanghai University Of Engineering Science. All Rights Reserved
# License: GNU General Public License v3.0.
# See LICENSE file in root directory.
# 
# Author: ligcox ligcox@birdiebot.top
# Date: 2022-09-01 18:04:40
# FilePath: /bubble/src/bubble_core/bubble_protocol/bubble_protocol/bcp_core.py
# LastEditors: ligcox ligcox@birdiebot.top
# LastEditTime: 2022-09-01 18:24:39

import rclpy

from bubble_protocol.dispatch import RobotAPI

def main(args=None):
    """BCP core entrance.
    """
    rclpy.init(args=args)
    bcp_core = RobotAPI()
    rclpy.spin(bcp_core)
    bcp_core.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()