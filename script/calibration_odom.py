#!/usr/bin/env python
# -*- coding: utf-8 -*-

from cyber_py import cyber
import time
import sys
import atexit
from gflags import FLAGS
from modules.localization.proto import localization_pb2
import socket
from cyber_py import cyber_time
from signal import signal, SIGPIPE, SIG_DFL
from io import BlockingIOError
import select

signal(SIGPIPE, SIG_DFL)

class OdomSocket(object):
    def __init__(self, node):
        self.odom = localization_pb2.LocalizationEstimate()
        self.terminating = False

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setblocking(False)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.address = ('127.0.0.1', 17788)
        self.server_socket.bind(self.address)
        self.server_socket.listen(128)

        self.client_list = [self.server_socket]

    def odom_callback(self, data):
        if self.terminating is True:
            return
        self.odom.CopyFrom(data)

    def socket_send(self):
        if self.terminating is True:
            return
        while True:
            try:
                readable, writable, errored = select.select(self.client_list, [], [])
                for s in readable:
                    if s is self.server_socket:
                        client_socket, addr = self.server_socket.accept()
                        self.client_list.append(client_socket)
                    else:
                        ra = s.recv(1024)
                        if ra:
                            # data = self.odom.SerializeToString()

                            header_timestamp_sec = self.odom.header.timestamp_sec
                            header_module_name = self.odom.header.module_name
                            header_sequence_num = self.odom.header.sequence_num
                            header_data = repr(header_timestamp_sec) + '&' + str(header_module_name) + '&' + repr(header_sequence_num) + '&'

                            pose_position_x = self.odom.pose.position.x
                            pose_position_y = self.odom.pose.position.y
                            pose_position_z = self.odom.pose.position.z
                            pose_orientation_x = self.odom.pose.orientation.qx
                            pose_orientation_y = self.odom.pose.orientation.qy
                            pose_orientation_z = self.odom.pose.orientation.qz
                            pose_orientation_w = self.odom.pose.orientation.qw
                            pose_data = repr(pose_position_x) + '&' + repr(pose_position_y) + '&' + repr(pose_position_z) + '&' +\
                                repr(pose_orientation_x) + '&' + repr(pose_orientation_y) + '&' + repr(pose_orientation_z) + '&' +\
                                    repr(pose_orientation_w)

                            data = header_data + pose_data

                            s.send(data)
                        else:
                            s.close()
                            self.client_list.remove(s)
            except BlockingIOError:
                pass
            
        self.server_socket.shutdown()
        self.server_socket.close()

    def shutdown(self):
        self.terminating = True


def main():
    node = cyber.Node("odom_socket")
    recorder = OdomSocket(node)
    atexit.register(recorder.shutdown)

    node.create_reader('/apollo/localization/pose', localization_pb2.LocalizationEstimate, 
                        recorder.odom_callback)
    recorder.socket_send()

if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()