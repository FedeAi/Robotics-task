//
// Created by federico on 04/05/22.
//
#include <ros/ros.h>
#include "task1/RosNodeTask1.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "task1_node");
    ros::NodeHandle nodeHandle("~");

    RosNodeTask1 rosPackageTemplate(nodeHandle);
    ros::spin();
    return 0;
}
