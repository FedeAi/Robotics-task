//
// Created by federico on 27/04/22.
//

#ifndef TASK1_ROSNODETASK1_H
#define TASK1_ROSNODETASK1_H

#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/JointState.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/TwistStamped.h" // TODO controllare di averla inclusa in cmake package
#include "task1/kinematic/WheelEncoderHelper.h"
#include "task1/kinematic/Kinematic.h"
#include "task1/odometry/Odometry.h"
#include "task1/kinematic/MecanumKinematic.h"

/**
 * RosTask1Node class handles the binding between ros and our custom algorithms, it gives the possibility to subscribe,
 * publish, reconfigure...
 *
 * @author Federico Sarrocco
 */
class RosNodeTask1 {

public:
    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
    RosNodeTask1(ros::NodeHandle &nodeHandle);

    /*!
   * Destructor.
   */
    virtual ~RosNodeTask1() = default;;

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS topic callback method. It receives wheel message
     * @param message the received message.
     */
    void wheelCallback(const sensor_msgs::JointState &message);

    /*!
     * ROS service server callback.
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if successful, false otherwise.
     */
    bool serviceCallback(std_srvs::Trigger::Request &request,
                         std_srvs::Trigger::Response &response);

    void publishSpeed();

    void publishOdom();

    //! ROS node handle.
    ros::NodeHandle &nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriber_;

    //! ROS topic name to subscribe to.
    std::string wheelSubscriberTopic_;

    //! ROS topic publisher
    ros::Publisher speedPub_;
    ros::Publisher odomPub_;

    //! ROS topic name to publish to
    std::string speedPublisherTopic_;
    std::string odomPublisherTopic_;

    //! ROS service server.
    ros::ServiceServer serviceServer_;

    //! helpers to keep track of each wheel speed
    WheelSpeedCalculator wheelFL_, wheelFR_, wheelRL_, wheelRR_;

    //! kinematic calculator
    MecanumKinematic kinematicCalculator_;

    //! odometry calculator
    Odometry odometryCalculator_;

    //! robot params
    double w_, l_, wheelRadius_, encoderCPR_, gearRatio_;

    //! helper template function to get ros param, or return a default value if the param is not found
    template<typename T1, typename T2>
    T2 getRequiredRosParam(ros::NodeHandle nh, std::string paramName,
                           T1 defaultValue, bool &has_failed) {
        T1 var;

        if (!nh.getParam(paramName, var)) {
            has_failed = true;
            std::cerr << paramName.c_str() << "-->";
            std::cerr << "using default value" << std::endl;
        }

        nh.template param(paramName, var, defaultValue);
        return T2(var);
    }

};


#endif //TASK1_ROSNODETASK1_H
