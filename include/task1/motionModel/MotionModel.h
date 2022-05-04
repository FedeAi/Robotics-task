//
// Created by federico on 04/05/22.
//

#ifndef TASK1_MOTIONMODEL_H
#define TASK1_MOTIONMODEL_H

/**
 * MotionModel class is an interface, it's used to derive different motion model Concrete classes
 * @author Federico Sarrocco
 */
class MotionModel {

public:
    /**
     * Struct defining a robot pose in x, y and theta
     */
    struct Pose {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;

        Pose() = default;

        Pose(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {};
    };

    /**
     * Struct defining a robot pose variation in dx, dy and dtheta
     */
    struct DPose {
        double dx = 0.0;
        double dy = 0.0;
        double dyaw = 0.0;

        DPose() = default;

        DPose(double dx, double dy, double dyaw) : dx(dx), dy(dy), dyaw(dyaw) {};
    };

    /**
     * Struct defining a control command in vl, vr and y0 rate
     */
    struct Control {
        double vl = 0.0; /** Front and rear left wheel average speed in rad/s */
        double vr = 0.0; /** Front and rear right wheel average speed in rad/s */
        double y0 = 0.0; /** Apparent baseline in m */

        Control() = default;

        Control(double vl, double vr, double y0) : vl(vl), vr(vr), y0(y0) {};
    };

    /**
     * Starting from a pose and given a control command and an elapsed time, obtain the resulting pose
     * Uses Euler Forward integration
     * @param old_pose starting pose
     * @param control control command given in the step
     * @param delta_time elapsed time from old pose to the final pose
     * @return final pose
     */
    static Pose updatePoseEuler(const Pose &old_pose, const Control &control, double delta_time);

    /**
     * Starting from a pose and given a control command and an elapsed time, obtain the resulting pose
     * Uses Runge-Kutta 4th order integration
     * @param old_pose starting pose
     * @param control control command given in the step
     * @param delta_time elapsed time from old pose to the final pose
     * @return final pose
     */
    static Pose updatePoseRK4(const Pose &old_pose, const Control &control, double delta_time);

private:
    /**
     * Get kn increments of pose for runge kutta
     * @param starting_pose initial state vector pose
     * @param control control command in the given step
     * @return get pose derivative for runge kutta
     */
    static DPose getF(const MotionModel::Pose &starting_pose, const MotionModel::Control &control);
};


#endif //TASK1_MOTIONMODEL_H
