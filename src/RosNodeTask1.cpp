//
// Created by federico on 27/04/22.
//
#include "task1/RosNodeTask1.h"
#include "task1/kinematic/MecanumKinematic.h"
#include "task1/filtering/LowPassFilter.h"
#include "task1/odometry/RungeKutta4.h"
#include "task1/odometry/Euler.h"

RosNodeTask1::RosNodeTask1(ros::NodeHandle &nodeHandle) : nodeHandle_(nodeHandle) {
    if (readParameters()) {
        ROS_ERROR("Could not read parameters.");
    }

    // attributes instantiation
    kinematicCalculator_ = MecanumKinematic(wheelRadius_, gearRatio_, l_, w_);
    wheelFL_ = WheelSpeedCalculator(encoderCPR_);
    wheelFR_ = WheelSpeedCalculator(encoderCPR_);
    wheelRL_ = WheelSpeedCalculator(encoderCPR_);
    wheelRR_ = WheelSpeedCalculator(encoderCPR_);

    odometryCalculator_ = Odometry(std::make_unique<Rungekutta4>(Rungekutta4()), startingOdom_.at(0),
                                   startingOdom_.at(1), startingOdom_.at(2));

    subscriber_ = nodeHandle_.subscribe(wheelSubscriberTopic_, 3, &RosNodeTask1::wheelCallback, this);
    speedPub_ = nodeHandle_.advertise<geometry_msgs::TwistStamped>(speedPublisherTopic_, 1);
    wheelsSpeedPub_ = nodeHandle_.advertise<task1::WheelsSpeed>(wheelsSpeedPublisherTopic_, 1);
    odomPub_ = nodeHandle_.advertise<nav_msgs::Odometry>(odomPublisherTopic_, 1);

    // start dynamic reconfigure server
    f = boost::bind(&::RosNodeTask1::dynamicResetParams, this, _1, _2);
    srv.setCallback(f);

    // start service server
    serviceServer_ = nodeHandle_.advertiseService("set_pose",
                                                  &RosNodeTask1::serviceCallback, this);

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
    // compute robot speed [w, vx ,vy] starting from wheel state
    kinematicCalculator_.computeKinematic(wheelSpeeds);
    // compute wheel speed starting from robot speed [w, vx ,vy]
    kinematicCalculator_.computeWheelsSpeed(kinematicCalculator_.getControl());

    //! odometry computing
    odometryCalculator_.updatePosition(kinematicCalculator_.getControl(), message.header.stamp.toSec());

    //! publishers
    publishSpeed();
    publishOdom();
    publishWheelSpeed();
}

void RosNodeTask1::publishSpeed() {
    geometry_msgs::TwistStamped message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = robotFrameID_;

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
    message.header.stamp = ros::Time::now();
    message.header.frame_id = odomFrameID_;

    message.pose.pose.position.x = odometryCalculator_.getPose().getPosition().x();
    message.pose.pose.position.y = odometryCalculator_.getPose().getPosition().y();
    message.pose.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, odometryCalculator_.getPose().getTheta());
    message.pose.pose.orientation.x = q.x();
    message.pose.pose.orientation.y = q.y();
    message.pose.pose.orientation.z = q.z();
    message.pose.pose.orientation.w = q.w();

    publishTFBaseLink();
    odomPub_.publish(message);
}

void RosNodeTask1::publishTFBaseLink() {
    if(!staticTransformPublished_){
        // do it just once
        publishTFOdom(odometryCalculator_.getPose().getPosition().x(), odometryCalculator_.getPose().getPosition().y(), odometryCalculator_.getPose().getTheta());
        staticTransformPublished_ = true;
    }
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odomFrameID_;
    transformStamped.child_frame_id = robotFrameID_;
    transformStamped.transform.translation.x = odometryCalculator_.getPose().getPosition().x();
    transformStamped.transform.translation.y = odometryCalculator_.getPose().getPosition().y();
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0,  odometryCalculator_.getPose().getTheta());
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tfBroadcasterBaseLink_.sendTransform(transformStamped);
}

void RosNodeTask1::publishTFOdom(double dx, double dy, double dtheta) {
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = odomFrameID_;
    transformStamped.child_frame_id = globalFrameID_;
    transformStamped.transform.translation.x = dx;
    transformStamped.transform.translation.y = dy;
    transformStamped.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0,  dtheta);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tfBroadcasterOdom_.sendTransform(transformStamped);
}

void RosNodeTask1::publishWheelSpeed() {
    task1::WheelsSpeed message;
    message.header.stamp = ros::Time::now();
    message.header.frame_id = robotFrameID_;

    message.rpm_fl = kinematicCalculator_.getWheelsControlSpeed().x() * 60 / M_PI / 2.0;
    message.rpm_fr = kinematicCalculator_.getWheelsControlSpeed().y() * 60 / M_PI / 2.0;
    message.rpm_rr = kinematicCalculator_.getWheelsControlSpeed().z() * 60 / M_PI / 2.0;
    message.rpm_rl = kinematicCalculator_.getWheelsControlSpeed().w() * 60 / M_PI / 2.0;

    wheelsSpeedPub_.publish(message);
}

bool RosNodeTask1::readParameters() {
    bool hasFailed = false;
    l_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/l",
                                             0.20327909544603728,
                                             hasFailed);
    w_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/w",
                                             0.1621234061413258,
                                             hasFailed);
    wheelRadius_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/r",
                                                       0.07221023879945607,
                                                       hasFailed);
    encoderCPR_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/CPR",
                                                      40.0,
                                                      hasFailed);
    gearRatio_ = getRequiredRosParam<double, double>(nodeHandle_, "robot/params/gearRatio",
                                                     5.0,
                                                     hasFailed);
    wheelSubscriberTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/subscriber_topic_wheel",
                                                                          "/wheel_states", hasFailed);

    speedPublisherTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/publisher_cmd_vel", "/cmd_vel",
                                                                         hasFailed);
    wheelsSpeedPublisherTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/publisher_wheels_vel", "/wheels_rpm",
                                                                         hasFailed);
    odomPublisherTopic_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/publisher_odom", "/odom",
                                                                        hasFailed);
    globalFrameID_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/global_frame_id", "world",
                                                                        hasFailed);
    robotFrameID_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/robot_frame_id", "base_link",
                                                                        hasFailed);
    odomFrameID_ = getRequiredRosParam<std::string, std::string>(nodeHandle_, "ros/odom_frame_id", "odom",
                                                                        hasFailed);

    startingOdom_ = getRequiredRosParam<std::vector<double>, std::vector<double>>(nodeHandle_, "robot/starting_odom",
                                                                                  {0.0006646061665378511,
                                                                                   0.02518487349152565,
                                                                                   0.004701722413301468},
                                                                                  hasFailed);
    return hasFailed;
}

void RosNodeTask1::dynamicResetParams(task1::SetConfig &config, u_int32_t level) {
    if (config.Integration_method == 0) {
        odometryCalculator_.setIntegrationMethod(std::make_unique<Euler>(Euler()));
        ROS_INFO("using Euler integration");
    }
    if (config.Integration_method == 1) {
        odometryCalculator_.setIntegrationMethod(std::make_unique<Rungekutta4>(Rungekutta4()));
        ROS_INFO("using Runge Kutta integration");
    }

    //Wheel filtering
    if (config.Wheel_speed_filtering == 0) {
        wheelFL_.setFilter(std::make_unique<Filter>(Filter()));
        wheelFR_.setFilter(std::make_unique<Filter>(Filter()));
        wheelRL_.setFilter(std::make_unique<Filter>(Filter()));
        wheelRR_.setFilter(std::make_unique<Filter>(Filter()));
        ROS_INFO("no filtering on wheel speed");
    }
    if (config.Wheel_speed_filtering == 1) {
        wheelFL_.setFilter(std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
        wheelFR_.setFilter(std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
        wheelRL_.setFilter(std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
        wheelRR_.setFilter(std::make_unique<LowPassFilter>(LowPassFilter(1.0, 1.0/15.0, 0)));
        ROS_INFO("low pass filter on wheel speed");
    }
}

bool RosNodeTask1::serviceCallback(task1::SetPoseService::Request &request, task1::SetPoseService::Response &response) {
    ROS_INFO("pose reset service called");
    // resetting tf odom
    publishTFOdom(request.x, request.y, request.theta);
    // resetting odometry pose
    odometryCalculator_.resetPose(Pose({0, 0}, 0));
    return true;
}

