//
// Created by federico on 27/04/22.
//
#include "task1/RosNodeTask1.h"
#include "task1/kinematic/MecanumKinematic.h"
#include "task1/filtering/LowPassFilter.h"
#include "task1/odometry/RungeKutta4.h"

RosNodeTask1::RosNodeTask1(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
    if (readParameters()) {
        ROS_ERROR("Could not read parameters.");
    }

    // attributes instantiation
    kinematicCalculator_ = MecanumKinematic(wheelRadius_, gearRatio_, l_, w_);
    wheelFL_ = WheelSpeedCalculator(encoderCPR_); //, std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
    wheelFR_ = WheelSpeedCalculator(encoderCPR_); //, std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
    wheelRL_ = WheelSpeedCalculator(
            encoderCPR_);    //, std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
    wheelRR_ = WheelSpeedCalculator(encoderCPR_); //, std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));

    odometryCalculator_ = Odometry(std::make_unique<Rungekutta4>(Rungekutta4()));

    subscriber_ = nodeHandle_.subscribe(wheelSubscriberTopic_, 1, &RosNodeTask1::wheelCallback, this);
    speedPub_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>(speedPublisherTopic_, 1);
    odomPub_ = nodeHandle_.advertise<nav_msgs::Odometry>(odomPublisherTopic_, 1);
    ROS_INFO("Succesfully launched node.");
}

void RosNodeTask1::wheelCallback(const sensor_msgs::JointState &message) {
    //! wheel speed computing
    wheelFL_.computeWheelSpeed(message.position.at(0), message.header.stamp.toSec());
    wheelFR_.computeWheelSpeed(message.position.at(1), message.header.stamp.toSec());
    wheelRL_.computeWheelSpeed(message.position.at(2), message.header.stamp.toSec());
    wheelRR_.computeWheelSpeed(message.position.at(3), message.header.stamp.toSec());

    //! kinematic computing
    Eigen::Vector4d wheelSpeeds = {wheelFL_.getAngularSpeed(), wheelFR_.getAngularSpeed(), wheelRR_.getAngularSpeed(),
                                   wheelRL_.getAngularSpeed()};
    kinematicCalculator_.computeKinematic(wheelSpeeds);

    //! odometry computing
    odometryCalculator_.updatePosition(kinematicCalculator_.getControl());

    publishSpeed();
    publishOdom();
}

void RosNodeTask1::publishSpeed() {
    geometry_msgs::TwistStamped message;
    message.header.stamp = ros::Time::now();  //todo frame id

    message.twist.linear.x = kinematicCalculator_.getLinearSpeed().x();
    message.twist.linear.y = kinematicCalculator_.getLinearSpeed().y();
    message.twist.linear.z = 0;

    message.twist.angular.x = 0;
    message.twist.angular.y = 0;
    message.twist.angular.z = kinematicCalculator_.getAngularSpeed();

    speedPub_.publish(message);
}

void RosNodeTask1::publishOdom() {
    nav_msgs::Odometry message;
    message.header.stamp = ros::Time::now(); // TODO frameid
    message.header.frame_id = "/world";
    message.child_frame_id = "/base_link";

    message.pose.pose.position.x = odometryCalculator_.getPose().getPosition().x();
    message.pose.pose.position.y = odometryCalculator_.getPose().getPosition().y();
    message.pose.pose.position.z = 0;

    message.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odometryCalculator_.getPose().getTheta());
    std::cout << odometryCalculator_.getPose().getTheta() << std::endl;

    odomPub_.publish(message);
}

bool RosNodeTask1::readParameters() {
    bool hasFailed = false;
    l_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/l",
                                             0.2,
                                             hasFailed);
    w_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/w",
                                             0.169,
                                             hasFailed);
    wheelRadius_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/r",
                                                       0.07,
                                                       hasFailed);
    encoderCPR_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/CPR",
                                                      42.0,
                                                      hasFailed);
    gearRatio_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/gearRatio",
                                                     5.0,
                                                     hasFailed);
    wheelSubscriberTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "subscriber_topic_wheel",
                                                                          "/wheel_states",
                                                                          hasFailed);

    speedPublisherTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "publisher_cmd_vel", "/cmd_vel",
                                                                         hasFailed);
    odomPublisherTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "publisher_odom", "/odom",
                                                                        hasFailed);
    return hasFailed;
}

