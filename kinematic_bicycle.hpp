#ifndef KINEMATIC_BICYCLE_HPP
#define KINEMATIC_BICYCLE_HPP

#include "casadi/casadi.hpp"
#include <chrono>
#include "sys/stat.h"
#include <fstream>

struct KinematicBicycleProperties {
    KinematicBicycleProperties(const double &_wb = 1.0) : wheel_base(_wb) {}

    ~KinematicBicycleProperties() {}

    double wheel_base;
};


class KinematicBicycle {
public:
    KinematicBicycle(const KinematicBicycleProperties &props);

    KinematicBicycle();

    ~KinematicBicycle() {}

    casadi::Function getDynamics() { return NumDynamics; }

    casadi::Function getOutputMapping() { return OutputMap; }

private:
    casadi::SX state;
    casadi::SX control;
    casadi::SX Dynamics;

    casadi::Function NumDynamics;
    casadi::Function OutputMap;
};




#endif // KINEMATIC_BICYCLE_HPP
