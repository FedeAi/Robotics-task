//
// Created by federico on 06/05/22.
//

#ifndef TASK1_EULER_H
#define TASK1_EULER_H

#include "Integrator.h"

class Euler : public Integrator {
    Pose updatePose(Pose startingPose, Control control, double deltaTime) override;
};

#endif //TASK1_EULER_H
