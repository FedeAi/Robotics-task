//
// Created by federico on 27/04/22.
//

#ifndef TASK1_ROSTASK1NODE_H
#define TASK1_ROSTASK1NODE_H

/**
 * RosTask1Node class handles the binding between ros and our custom algorithms, it gives the possibility to subscribe,
 * publish, reconfigure...
 *
 * @author Federico Sarrocco
 */
class RosTask1Node {

public:
    /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
    RosTask1Node(ros::NodeHandle& nodeHandle);

    /*!
   * Destructor.
   */
    virtual ~RosPackageTemplate();

private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS topic callback method.
     * @param message the received message.
     */
    void topicCallback(const sensor_msgs::Temperature& message);

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
    std::string subscriberTopic_;

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


#endif //TASK1_ROSTASK1NODE_H
