import localization_pb2
import socket
import time
from signal import signal, SIGPIPE, SIG_DFL

import rospy
from geometry_msgs.msg import TransformStamped

signal(SIGPIPE, SIG_DFL)

tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

server_ip = "127.0.0.1"
server_port = 17788

tcp_client_socket.connect((server_ip, server_port))

send_data = "hello"

rospy.init_node('node', anonymous = True)
pub = rospy.Publisher('/odom_data', TransformStamped, queue_size = 200)
rate = rospy.Rate(200)
msg = TransformStamped()

# while True:
while not rospy.is_shutdown():
    tcp_client_socket.send(send_data)

    recvData = tcp_client_socket.recv(1024)
    # if len(recvData) < 261:
    #     continue
    # print(recvData)
    try:
        # target = localization_pb2.LocalizationEstimate()
        # target.ParseFromString(recvData)
        # print(target)
        # print(recvData)
        # print("-------------------")
        # time.sleep(0.05)


        # msg.header.seq = target.header.sequence_num
        # msg.header.frame_id = target.header.module_name
        # msg.header.stamp = rospy.Time.from_sec(target.header.timestamp_sec)

        # msg.child_frame_id = 'rslidar'

        # msg.transform.translation.x = target.pose.position.x
        # msg.transform.translation.y = target.pose.position.y
        # msg.transform.translation.z = target.pose.position.z

        # msg.transform.rotation.x = target.pose.orientation.qx
        # msg.transform.rotation.y = target.pose.orientation.qy
        # msg.transform.rotation.z = target.pose.orientation.qz
        # msg.transform.rotation.w = target.pose.orientation.qw

        # header_timestamp_sec = recvData
        suffix = '&'
        data_list = recvData.split(suffix, 9)

        msg.header.seq = float(data_list[2])
        msg.header.frame_id = data_list[1]
        msg.header.stamp = rospy.Time.from_sec(float(data_list[0]))
        msg.child_frame_id = 'rslidar'
        msg.transform.translation.x = float(data_list[3])
        msg.transform.translation.y = float(data_list[4])
        msg.transform.translation.z = float(data_list[5])
        msg.transform.rotation.x = float(data_list[6])
        msg.transform.rotation.y = float(data_list[7])
        msg.transform.rotation.z = float(data_list[8])
        msg.transform.rotation.w = float(data_list[9])

        pub.publish(msg)
    except:
        pass
    rate.sleep()

tcp_client_socket.close()

