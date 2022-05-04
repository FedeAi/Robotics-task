//
// Created by federico on 27/04/22.
//
#include "task1/RosNodeTask1.h"

RosNodeTask1::RosNodeTask1(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle){
    if (readParameters()) {
        ROS_ERROR("Could not read parameters.");
    }

    subscriber_ = nodeHandle_.subscribe(subscriberTopicWheel_, 1, &RosNodeTask1::wheelCallback, this);
    //publisher_ = nodeHandle_.advertise<sensor_msgs::Image>(publisherTopicCanny_,1);
    ROS_INFO("Succesfully launched node.");
}

void RosNodeTask1::wheelCallback(const sensor_msgs::JointState &message) {
    // [front_left, front_right, rear_left, rear_right]
}

bool RosNodeTask1::readParameters() {
    bool hasFailed = false;
    subscriberTopicWheel_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "subscriber_topic_wheel", "/wheel_states",
                                                                     hasFailed);

    return hasFailed;
}