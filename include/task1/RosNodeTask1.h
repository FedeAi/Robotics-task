//
// Created by federico on 27/04/22.
//

#ifndef TASK1_ROSNODETASK1_H
#define TASK1_ROSNODETASK1_H
#include <std_srvs/Trigger.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"

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
    RosNodeTask1(ros::NodeHandle& nodeHandle);

    /*!
   * Destructor.
   */
    virtual ~RosNodeTask1();

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
    void wheelCallback(const sensor_msgs::JointState& message);

    /*!
     * ROS service server callback.
     * @param request the request of the service.
     * @param response the provided response.
     * @return true if successful, false otherwise.
     */
    bool serviceCallback(std_srvs::Trigger::Request& request,
                         std_srvs::Trigger::Response& response);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! ROS topic subscriber.
    ros::Subscriber subscriber_;

    //! ROS topic name to subscribe to.
    std::string subscriberTopicWheel_;

    //! ROS service server.
    ros::ServiceServer serviceServer_;

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
