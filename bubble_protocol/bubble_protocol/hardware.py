
# Copyright (c)  2022 Birdiebot R&D Department
# Shanghai University Of Engineering Science. All Rights Reserved
# License: GNU General Public License v3.0.
# See LICENSE file in root directory.
#
# Author: ligcox ligcox@birdiebot.top
# Date: 2022-08-29 20:52:49
# FilePath: /bubble/src/bubble_core/bubble_protocol/bubble_protocol/hardware.py
# LastEditors: ligcox ligcox@birdiebot.top
# LastEditTime: 2022-09-01 22:02:34
'''
Robot communication dispatch layer,
The module sets the serial port rules, 
and analyzes the communication data 
between the onboard computer and MCU.
'''

import queue
import struct
import time

import serial

from bubble_protocol.protocol import *


RX_BUFFER_MAX_SIZE = 500
TX_BUFFER_MAX_SIZE = 500


class RobotSerial(serial.Serial):
    def __init__(self, name, *, port="/dev/ttyUSB0", baudrate=921600, timeout_T=0):
        '''UART device initialization

        Parameters
        ------------
            port: `string`
                Com port path
            baudrate: `int`
                Serial port baudrate
            timeout_T: `int`
                Timeout

        Returns
        -----------
            `serial.Serial`
            Class return a serial port object.
        '''
        self.init_device(port, baudrate, timeout_T)
        self.init_protocol(name)
        self.status = STATUS
        self.serial_done = False
        self.realtime_pub = dict()

    def init_device(self, port: str, baudrate: int, timeout_T: int) -> None:
        '''UART device initialization parameters setting

        Parameters
        ------------
            port: `str`
                Com port path
            baudrate: `int`
                Serial port baudrate
            timeout_T: `int`
                Timeout
        '''
        super().__init__(port=port, baudrate=baudrate, timeout=timeout_T)
        self.rx_count = 0
        self.tx_count = 0
        self.reset_rx_buffer()

    def init_protocol(self, name: str) -> None:
        '''Initialize protocol frame param used robot name.

        Parameters
        ------------
            name: `str`
                robot name
        '''
        BCP_TX_FRAME.d_addr = D_ADDR[name]
        BCP_TX_FRAME.INFO = bytearray([HEAD, D_ADDR[name]])
        print("BCP_FRAME set d_addr: {}".format(BCP_TX_FRAME.d_addr))
        print("BCP_FRAME set INFO: {}".format(BCP_TX_FRAME.INFO))
        print("now init {} robot completed!".format(name))
        self.tx_buffer = queue.Queue()
        self.rx_buffer = queue.Queue()

    def send(self, data: bytearray) -> None:
        '''Write string to UART.

        Parameters
        ------------
        data: `bytearray`
            The data stream to be send
        '''
        # print(data)
        self.write(data)
        self.tx_count += 1

    def reset_rx_buffer(self) -> None:
        '''Reset receive buffer
        '''
        self.current_packet = BCP_FRAME()
        self.rx_status = 0
        self.rx_datalen = 0

    def rx_function(self) -> None:
        '''The data in the buffer is parsed periodically, 
            the function will put BCP freme to ``current_packet`` if data is parsed successfully. 
        '''
        try:
            rx_bytes = self.readall()
        except serial.SerialException as e:
            print("device reports readiness to read but returned no data")
            return
        except TypeError as e:
            print("disconnect occured")
            return

        # print([hex(i) for i in rx_bytes])
        for rx_byte in rx_bytes:
            if self.rx_status == 0:  # wait HEAD
                if rx_byte == HEAD:
                    self.current_packet.setData(rx_byte)
                    self.rx_status = 1
            elif self.rx_status == 1:  # wait D_ADDR
                # for normal:
                if rx_byte == D_ADDR["mainfold"]:
                    # for debug:
                    # if rx_byte ==  D_ADDR["standard"]:
                    self.current_packet.setData(rx_byte)
                    self.rx_status = 2
                else:
                    self.reset_rx_buffer()
            elif self.rx_status == 2:  # wait ID
                self.current_packet.setData(rx_byte)
                self.rx_status = 3
            elif self.rx_status == 3:  # wait LEN
                self.current_packet.setData(rx_byte)
                self.rx_status = 4
            elif self.rx_status == 4:  # wait DATA
                self.current_packet.setData(rx_byte)
                self.rx_datalen += 1
                if self.rx_datalen >= self.current_packet.info[LEN_POSE]:
                    self.rx_status = 5
                    self.current_packet.combineCheck()
            elif self.rx_status == 5:  # wait SUM_CHECK
                if rx_byte == self.current_packet.info[SUM_CHECK_POSE]:
                    self.rx_status = 6
                else:  # check fail
                    self.reset_rx_buffer()
            elif self.rx_status == 6:  # wait ADD_CHECK
                if rx_byte == self.current_packet.info[ADD_CHECK_POSE]:
                    self.rx_buffer.put(copy.deepcopy(self.current_packet.info))
                    # print(self.current_packet)
                self.reset_rx_buffer()

    def onboard_data_analysis(self, current_packet: BCP_FRAME) -> None:
        '''
        Parameters
        ------------
        current_packet: `BCP_FRAME`
            Update robot status by successfully parsed BCP frame.
        '''
        def getFrameFmt(info_list: list) -> str:
            '''
            Parameters
            ------------
            info_list: `list`
                Gets the type of  BCP frame to parse.

            Returns
            -----------
            `str`
                 A format string that needs to be passed to ``struct`` moudle for parsing
            '''
            return "<"+"".join([info_list[key][IDX_BCP_TYPE] for key in info_list])

        def getFrameRatio(info_list):
            '''Gets the ratio of frame data
            Parameters
            ------------
            info_list: `list`
                Gets the ratio of  BCP frame to parse.

            Returns
            -----------
            `list`
                A list of all parsed radio.
            '''
            return [info_list[key][IDX_BCPID_RATIO] for key in info_list]

        for key in STATUS:
            # print([hex(i) for i in current_packet])
            # unpack packet id
            if self.serial_done is False:
                return
            if current_packet[ID_POSE] == ID[key][IDX_BCPID]:
                data_info = ID[key][IDX_BCP_DETAIL]
                # print("===POSE====", DATA_POSE, SUM_CHECK_POSE)
                # print("===data_info====", getFrameFmt(data_info))
                unpack_info = struct.unpack(getFrameFmt(
                    data_info), current_packet[DATA_POSE: SUM_CHECK_POSE])
                ratio_list = getFrameRatio(data_info)
                res = list(
                    map(lambda info, ratio: info/ratio, unpack_info, ratio_list))
                for idx, detail_key in enumerate(self.status[key]):
                    self.status[key][detail_key][IDX_VAL] = res[idx]
                for pub_key in self.realtime_pub:
                    if key == pub_key and self.realtime_pub[pub_key] is not None:
                        self.realtime_pub[pub_key]()
                # print(self.status["manifold_ctrl"],self.status["gimbal"])
                return
        print(f"Received a vaild frame, but not defined by robot status.\n\
                BCP core refused to process this frame data.\n\
                process time:{time.time()}, current packtet id:{current_packet[ID_POSE]}, \
                packtet detail: {[i for i in current_packet]}")
        return

    def process(self):
        '''Thread handing functions. 
        The buffer data is periodically taken out by timer 
        for updating robot status or sending to MCU.

        Parameters
        ------------

        Returns
        -----------
        '''
        rx_buffer_size = self.rx_buffer.qsize()
        tx_buffer_size = self.tx_buffer.qsize()
        for _ in range(min(rx_buffer_size, RX_BUFFER_MAX_SIZE)):
            self.onboard_data_analysis(self.rx_buffer.get())
        for _ in range(min(tx_buffer_size, TX_BUFFER_MAX_SIZE)):
            self.send(self.tx_buffer.get())

    def setFrameData(self, frame: BCP_TX_FRAME, info: list, data: OrderedDict):
        frame_data_fmt = ""
        frame_data_list = []
        for index, key in enumerate(data):
            frame_data_fmt += data[key][IDX_BCP_TYPE]
            # print(info,data)

            frame_data_list.append(
                int(info[index] * data[key][IDX_BCPID_RATIO]))

        # cvt data to little-endian send  to uart
        # defined by protocol, send data are must a int numeber
        # cvt data to int before send
        res_data = struct.pack("<"+frame_data_fmt, *frame_data_list)
        # set LEN
        frame.setData(len(res_data))
        # set DATA
        for itm in res_data:
            frame.setData(itm)
        # set check data
        frame.combineCheck()

    def send_data(self, name: str, info: list):
        '''Convert data that needs to be send to MCU into BCP frame datas.

        Parameters
        ------------
        name: `str`
            robot name.
        info: `list`
            A list of data to send.
        '''
        frame = BCP_TX_FRAME()
        detail = ID[name]
        # set ID
        frame.setData(detail[IDX_BCPID])
        # set frame data
        self.setFrameData(frame, info, detail[IDX_BCP_DETAIL])

        self.tx_buffer.put(copy.deepcopy(frame.getData()))
        # print("UART send data {}, now transmit count: {}".format([hex(i) for i in frame.getData()], self.tx_count))
        # print("UART send data {}, now transmit count: {}".format([i for i in frame.getData()], self.tx_count))
