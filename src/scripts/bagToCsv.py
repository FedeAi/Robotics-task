import threading

import message_filters
import pandas as pd
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

BAG_NAME = "bags/bag4"
df = pd.DataFrame(columns=["x", "y", "theta", "eFL", "eFR", "eRR", "eRL", "timestamp"])


def callback(pose: PoseStamped, encoder: JointState):
    global df
    data = {}

    data["x"] = pose.pose.position.x
    data["y"] = pose.pose.position.y
    data["theta"] = pose.pose.orientation.z

    data["eFL"] = encoder.position[0]
    data["eFR"] = encoder.position[1]
    data["eRL"] = encoder.position[2]
    data["eRR"] = encoder.position[3]

    data["timestamp"] = encoder.header.stamp.to_sec()

    df = df.append(data, ignore_index=True)


def listener():

    rospy.init_node("bagToCsv")

    pose_sub = message_filters.Subscriber('/robot/pose', PoseStamped)
    encoder_sub = message_filters.Subscriber('/wheel_states', JointState)

    ts = message_filters.ApproximateTimeSynchronizer([pose_sub, encoder_sub], 10, 0.01)
    ts.registerCallback(callback)

    rospy.spin()


def saveData():
    global timer
    print("saving data to file")
    df.to_csv(BAG_NAME + ".csv")
    timer = threading.Timer(3, saveData)
    timer.start()

if __name__ == '__main__':
    global timer
    timer = threading.Timer(3, saveData)
    timer.start()
    listener()

