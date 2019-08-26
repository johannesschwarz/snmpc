#ifndef SIMPLE_BICYCLE_HPP
#define SIMPLE_BICYCLE_HPP

#include "casadi/casadi.hpp"
#include <chrono>
#include "sys/stat.h"
#include <fstream>
#include "wheeldyn_bicycle.h"


struct SimpleBicycleProperties
{
    SimpleBicycleProperties(const double &_wb = 1.0) : wheel_base(_wb){}
    ~SimpleBicycleProperties(){}
    double wheel_base;
};

/** Relatively simple mobile model : tricycle */
class SimpleBicycle
{
public:
    SimpleBicycle(const WheelDynBicycleProperties &props);
    SimpleBicycle();
    ~SimpleBicycle(){}

    casadi::Function getDynamics(){return NumDynamics;}
    casadi::Function getOutputMapping(){return OutputMap;}

    casadi::Function TraceFunction;
private:
    casadi::SX state;
    casadi::SX control;
    casadi::SX Dynamics;

    casadi::Function NumDynamics;
    casadi::Function OutputMap;
};

#endif // SIMPLE_BICYCLE_HPP
