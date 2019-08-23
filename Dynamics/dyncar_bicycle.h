#ifndef DYNCAR_BICYCLE_H
#define DYNCAR_BICYCLE_H

#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct __car_state {

  int state;   // is equal 1 when car is moving ; state = 0 when object DynamicCar being created

  double psi;  // yaw of the car // in world coordinate system
  double x;  //
  double y;  // x,y of the Center of Gravitaty (CG) // in world coordinate system

  double xc; // analogious coodinates x,y for Center of Rear axe of car (CR)
  double yc;  // in world coordinate system

  double Vx;   //Velocity of CG in coordinate system related to car:OX pointing along central axe of the car; OY pointing to left
  double Vy;  // So, Vx - longitudal speed, Vy - lateral speed of the car

  double Vdes; // Desired longitudal speed from Path Matcher (Control) // (only for logging)

  double Om;  // angular velocity of car in world coordinate system

  double dVxdt; // d(Vx)/dt
  double dVydt; // d(Vy)/dt

  double dOmdt; // d(Om)/dt

  double OmWr; // rotational speed of rear wheel
  double OmWf; // rotational speed of forward wheel

  double dOmWfdt; // d(OmWf)/dt
  double dOmWrdt; // d(OmWr)/dt

  // Diagnosis
  int callcount;
  int initialized;

  // Control data
  double steer_alpha_req; // required steering angle from Control
  double steer_alpha;  // steering angle (angle of turning of forward wheel)
  double accel;        // desired acceleration (decceleration) for car from Control


  //Forces

  double Nr;// Load on rear wheel
  double Nf;// Load on forward wheel

  double Fdrag; // drag force acting on car
  double Fdrag_PM; // evaluated drag force to have to be added to massa*accel force which is requested by Control

  double FXreq; // whole force which being requested from engines/brakes (= massa*accel + Fdrag_PM)

  double FXr_act;  // Requested Force for rear engines (actuators)
  double FXf_act;  // requested force for forward engines (distributed from Fxreq in correspondence to drive distribution)

  double FXr_req;  // Force for rear wheel (from engines and brakes)
  double FXf_req;  // force for forward wheel // These forces are requested from tyres in longitudal directional

  double FYr_req;  // these forces    are requested   // rear
  double FYf_req;  // from tyres in lateral direction // forward

  double FXr;      // real forces acting on tyres // rear
  double FXf;      // in longitudinal direction    // forward

  double FYr; // real forces acting on tyres // rear
  double FYf; // in lateral direction        // forward

  // brakes: Fbrake = car_par.n_brakes * car_state.B_pressure *(
  //                      (car_par.kBrake_f / car_par.frontWheelRad) +  (car_par.kBrake_f / car_par.frontWheelRad) )
  //

  double B_pressure; // Pressure for brakes
  double B_pressure_req; // Requested pressure

  // wheels

  double slipAng_r; // lateral slip // rear
  double slipAng_f; // angles       // forward

  double long_slip_r; // longitudal slip // rear
  double long_slip_f; // angles          // forward

  double lat_slip;

} car_state_t;
#ifdef __cplusplus
};
#endif

#ifdef __cplusplus
#include <Eigen/Eigen>

// ------ Control structures
// --------------------------------------------------------------------------------------------//

struct CulcParameters {

  int fl_drag_taking; // if 1 then drag forces being taked into account
  int fl_Long_Circ;  // 0 || 1 || 2 ; 2 is default value (combined slip mode)
  int fl_static_load; // 1 for static case of loads on rear and forward wheel
  int fl_RunKut; // 1 for Runge Kutta integration; 0 for Local Grid Refinement (more expensive) // 1 is default
  int nsteps; // sampling for integration on [t,t+dt] interval // 100 is default
  int fl_act_low; // Lawes for actuators;// 2 is default

};

struct CarParameters {
  //--- kinematic ----//

  double a; // distance from CG to forward wheel
  double b; // distance from CG to rear wheel
  double L; // wheel base (= a+b)


  double max_steer; // max turning angle for forward wheel

  double max_accel;  // maximal value of requested acceleration/decceleration

  //-- Dynamic ----------/
  double drive_distrib;  //  drive distribution: 0 - RWD, 1 - FWD, 0.5 - AWD etc.
  double mass;           // mass of car
  double moment_inertia; // yaw moment of inertia for car
  double CGvert;         // high of CG
  double mu;             // mu (friction coefficient)

  // stiftness of tyres in lateral direction (for 2 wheels):
  double Cy_r; // rear
  double Cy_f; // forward

  // stiftness of tyres in longitudal direction (for 2 wheels):
  double Cx_r; // rear
  double Cx_f; // forward

  // actuators -----------/

  double rearWheelRad; // radiuses of // rear
  double frontWheelRad; // wheels,m   // forward

  double maxPowerEngine;   // for one engine, watt
  double maxTorqueEngine;  // for one engine, on wheels, N*m
  double n_engines;        // number of engines for rear and forward wheels (this numbers are the same for rear and forward wheel)

  // torque curve
  double Om_maxTorque; // parameters of            //
  double Om_nullTorque; // torque curve for engines//

  // Brakes parameters:
  double maxBrakePressure;      // Bar
  double maxGradBrakePressure;  // Bar/s
  double kBrake_f;              // N*m/Bar
  double kBrake_r;              //
  double n_brakes;


  double maxGradSteer;  // rad/s // speed of turning of forward wheel (speed of steering changes)

  //------------------/

  // drag parameters --/

  double rollingResistance;               // [N] Rolling resistance //
  double rollingResistanceSpeedDepended;  // [N*s/m] Rolling resistance
  double Cxx;                             // Coefficient of aero drag

  // -- in old PM:
  // turningDrag   = m*b/(a+b)*abs(yawRate*vxCG)*abs(tan(delta));
  // aeroDrag      = C_xx * vxCG * vxCG;
  // inclinedDrag  = -m*g*sin(grade);  %negative sign due to GPS convention for
  // grade FxDrag = turningDrag + rollingResistance + aeroDrag + inclinedDrag;
  //-------------------/

  //-----------------/

  double fWIn;  // moment of inertia fo front train
  double rWIn;  // -''- rear
};

struct DynamicPar {

  CulcParameters culc_par;

  CarParameters car_par;

  int fl_Dynamic;    //  not used
  int fl_Dynamic_PM; // not used

  DynamicPar() {
    fl_Dynamic = 1;
    fl_Dynamic_PM = 1;

    // Parameters of culculations
    culc_par.fl_drag_taking = 1;
    culc_par.fl_Long_Circ = 2;
    culc_par.fl_static_load = 0;
    culc_par.fl_RunKut = 1;
    culc_par.nsteps = 100;
    culc_par.fl_act_low = 2;

    //----------------------------------------
    // default kinematic and dynamic parameters
    car_par.a = 1.4;  //1.73;
    car_par.b = 1.4;  //1.168;  // 1.35;
    car_par.L = car_par.a + car_par.b;
    car_par.max_steer = 22.0 * M_PI / 180.0;  // 52.0*M_PI/180.0;

    //-----------------------------
    car_par.mu = 1.0;
    car_par.max_accel = 8.0;  // 50.0
    car_par.CGvert = 0.4;
    car_par.mass = 1350; //1196.0;
    car_par.moment_inertia = 1260.0;

    //------------------------------------
    car_par.Cy_f = 226000.0;
    car_par.Cy_r = 240000.0;
    car_par.Cx_f = 376000.0;
    car_par.Cx_r = 284000.0;

    //------------------------------------
    car_par.drive_distrib = 0.0;  // 0.5 = AWD, 1.0 = FWD, 0.0 = RWD
    car_par.frontWheelRad = 0.31;
    car_par.rearWheelRad = 0.35;
    car_par.maxPowerEngine = 135000.0;
    car_par.maxTorqueEngine = 200.0 * 6.25;
    car_par.n_engines = 2.0;
    car_par.Om_maxTorque = 192.68;
    car_par.Om_nullTorque = 201.06;
    //
    car_par.maxBrakePressure = 60.0;
    car_par.maxGradBrakePressure = 250.0;
    car_par.kBrake_f = 70.0;
    car_par.kBrake_r = 70.0;
    car_par.n_brakes = 1.0;
    car_par.maxGradSteer = 8.22;

    //------------------------------------
    car_par.rollingResistance = 0.0;//600.0;
    car_par.rollingResistanceSpeedDepended = 0.0;
    car_par.Cxx = 1.0;

    //-----------------------------------------------

    car_par.fWIn = 1.42;
    car_par.rWIn = 2.06;
  }
};

//----- Dynamic Car
//---------------------------------------------------------------------------------------------------//

class DynamicCar {
 protected:
  void newFXreq();

  void getDrag();

  void getVerticalLoads();

  void newLongForce();

  void newLatForce();

  void analize_Fdrag(double& dVxdt);

  void evolutionControl(double dt);

  void combinedSlip();

  double brushModel_combined(double slip_angle, double Fz);

  void integrNav(double dt);

  void integrNavkinematictest(double dt);

  void addToMPClog(double dtk);

  void addToLog(double dt);

  double analize_dt(double t);

 public:
  DynamicCar(Eigen::Vector3d init_navig);
  virtual ~DynamicCar();

  int fl_first_call;
  // std::ofstream fileDyn;

  double in_time;
  double d_time;
  int fl_fileDyn;

  //  std::ofstream fileMPClog;

  double timeMPClog;
  int fl_MPClog;

  void dynamicNav_ev(double steer, double accel_des, double dt, int reverse_fl,
                     double speed_des);

  void init_nav(double x, double y, double psi, double v);

  int fl_reverse;
  double fl_AccBrkMode;
  //-----------------------------

  void nav_to_cog(int fl_reverse);

  void getAccBrkMode();
  void getDrag_PM();

  double brushModel_solver(double FY, int fl_forwardWheel, int& fl_OK);

  double brushModel(double slip_angle, double Fz, double Ctyre);

  void getLatSlip();

  //-----------------------------

  CulcParameters culc_par;
  CarParameters car_par;

  car_state_t car_state;
};
#endif  // __cplusplus__

#ifdef __cplusplus
extern "C" {
#endif

void initModel(double x, double y, double psi, double v);
void stepFunc(double steer, double accel_des, double dt, int reverse_fl,
              double speed_des, car_state_t* __dcar);

#ifdef __cplusplus
};
#endif

#endif  // DYNCAR_BICYCLE_H
