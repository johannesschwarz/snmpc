#include <iostream>
#include "dyncar_bicycle.h"

//-/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//---/////////////////////////////////////////////////

//-////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-//////////////////////////////////////////

// DYNAMIC

// CAR

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DynamicCar::~DynamicCar() {}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

DynamicCar::DynamicCar(Eigen::Vector3d init_navig) {

fl_first_call = 1;
  //==========================================

  // Parameters of culculations
  culc_par.fl_drag_taking = 1;
  culc_par.fl_Long_Circ = 2;
  culc_par.fl_static_load = 0;
  culc_par.fl_RunKut = 0;
  culc_par.nsteps = 100;
  culc_par.fl_act_low = 0;

  //----------------------------------------
  // default kinematic and dynamic parameters
  car_par.a = 1.4; //1.73;
  car_par.b = 1.4; //1.168;  // 1.35;
  car_par.L = car_par.a + car_par.b;
  car_par.max_steer = 22.0 * M_PI / 180.0;  // 52.0*M_PI/180.0;

  //-----------------------------
  car_par.mu = 1.0;
  car_par.max_accel = 8.0;  // 50.0
  car_par.CGvert = 0.4;
  car_par.mass = 1350; //1196.0;
  car_par.moment_inertia = car_par.mass * car_par.a * car_par.b; //1260.0;

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
  car_par.rollingResistance = 600.0;
  car_par.rollingResistanceSpeedDepended = 0.0;
  car_par.Cxx = 1.0;

  //-----------------------------------------------

  car_par.fWIn = 1.42;
  car_par.rWIn = 2.06;


  //-----------------------------------------------
  // initial state ------------------------------//

  fl_reverse = 0;

  car_state.state = 0;

  car_state.xc = init_navig(0);
  car_state.yc = init_navig(1);
  car_state.psi = init_navig(2);

  nav_to_cog(0);

  car_state.Vx = 0.0;
  car_state.Vy = 0.0;
  car_state.Om = 0.0;

  car_state.accel = 0.0;
  car_state.steer_alpha = 0.0;
  car_state.steer_alpha_req = 0.0;

  car_state.dVxdt = 0.0;
  //   car_state.dVydt =  0.0;
  //   car_state.dOmdt =  0.0;

  car_state.slipAng_f = 0.0;
  car_state.slipAng_r = 0.0;

  car_state.FYf = 0.0;
  car_state.FYr = 0.0;

  car_state.FXf = 0.0;
  car_state.FXr = 0.0;

  car_state.FXf_req = 0.0;
  car_state.FXr_req = 0.0;

  car_state.B_pressure = 0.0;
  car_state.B_pressure_req = 0.0;

  car_state.OmWf = 0.0;
  car_state.OmWr = 0.0;

  car_state.long_slip_f = 0.0;
  car_state.long_slip_r = 0.0;

  //  car_state.initialized += 1;
  car_state.callcount = 0;
  //===================================================

  //------------------------------------

  fl_fileDyn = 0;
  // if(fl_fileDyn > 0)
  // fileDyn.open("Dyn.txt");
  in_time = 0.0;
  d_time = 0.0;

  fl_MPClog = 0;
  // if(fl_MPClog > 0)
  // fileMPClog.open("MPClog.txt");
  timeMPClog = 0.0;
}

//--------------------------------------------------------------------------------------

//-//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//+++++++++++++++++
void DynamicCar::newFXreq() {
  if (fabs(car_state.accel) < 0.00001) car_state.accel = 0.0;

  if (fabs(car_state.accel) > car_par.max_accel)
    car_state.accel =
        car_par.max_accel * (car_state.accel / fabs(car_state.accel));

  car_state.FXreq = car_par.mass * car_state.accel;

  // FXreq being force which one requests from car

  if ((fl_AccBrkMode > 0) || (culc_par.fl_act_low < 2)) {
    if (car_par.drive_distrib < 0.0) car_par.drive_distrib = 0.0;

    if (fabs(car_par.drive_distrib) > 1.0) car_par.drive_distrib = 1.0;

    if (car_par.drive_distrib < 0.01) {
      car_state.FXr_act = car_state.FXreq;
      car_state.FXf_act = 0.0;

    } else if (car_par.drive_distrib > 0.99) {
      car_state.FXf_act = car_state.FXreq;
      car_state.FXr_act = 0.0;
    } else {
      car_state.FXf_act = car_par.drive_distrib * car_state.FXreq;
      car_state.FXr_act = (1.0 - car_par.drive_distrib) * car_state.FXreq;
    }

    car_state.B_pressure_req = 0.0;
  } else {
    car_state.FXf_act = 0.0;
    car_state.FXr_act = 0.0;

    car_state.B_pressure_req = (fabs(car_state.FXreq) / car_par.n_brakes) /
                               ((car_par.kBrake_f / car_par.frontWheelRad) +
                                (car_par.kBrake_r / car_par.rearWheelRad));

    if (car_state.B_pressure_req > car_par.maxBrakePressure)
      car_state.B_pressure_req = car_par.maxBrakePressure;
  }
}
//-----------------------------------------------------------------------------------------
//-//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//+++++++++++++++++

void DynamicCar::getDrag() {
  car_state.Fdrag = 0.0;

  if (culc_par.fl_drag_taking > 0) {
    car_state.Fdrag =
                car_par.Cxx * car_state.Vx * car_state.Vx;

    if (car_state.Vx < 0.0) car_state.Fdrag *= -1.0;
  }
}

//-----------------------------------------------------------------------------------------

//-//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//+++++++++++++++++

void DynamicCar::getDrag_PM() {
  car_state.Fdrag_PM = 0.0;

  return;

  // turn resistance   //-----------------------------------------

  double acc_turn =
      (car_par.b / car_par.L) * car_state.Vdes * (car_state.Vdes / car_par.L) *
      tan(car_state.steer_alpha_req) * sin(car_state.steer_alpha_req);

  if (fl_reverse > 0) acc_turn *= -1.0;

  car_state.accel += acc_turn;

  if (culc_par.fl_drag_taking > 0) {
    car_state.Fdrag_PM =
        car_par.rollingResistance +
        car_par.rollingResistanceSpeedDepended * fabs(car_state.Vdes) +
        car_par.Cxx * car_state.Vdes * car_state.Vdes;

    if (fl_reverse > 0) car_state.Fdrag_PM *= -1.0;

    car_state.accel += (car_state.Fdrag_PM / car_par.mass);

    // one has to take into account different behaviours of Fdrag in the cases
    // of acceleration and braking. !! Fdrag_PM is usually culculated by PM.
    // Here is located such evaluation. After Fdrag_PM having evaluated the
    // car_state.accel should be corrected
  }
}
//-----------------------------------------------------------------------------------------

//-//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//+++++++++++++++++
void DynamicCar::getVerticalLoads() {

    double Trq_tyres = (car_state.FXr + car_state.FXf*cos(car_state.steer_alpha) - car_state.FYf*sin(car_state.steer_alpha))*car_par.CGvert;
    double weight = 9.89*car_par.mass;

    if(culc_par.fl_static_load > 0)
        Trq_tyres = 0.0;
    else if (culc_par.fl_static_load < 0)
        Trq_tyres = car_state.dVxdt*car_par.mass*car_par.CGvert;

    car_state.Nr = (weight*car_par.a + Trq_tyres)/car_par.L;

    if(car_state.Nr <= 0.0)
        car_state.Nr = 0.001;

    car_state.Nf = (weight*car_par.b - Trq_tyres)/car_par.L;

    if(car_state.Nf <= 0.0)
        car_state.Nf = 0.001;
}

//-----------------------------------------------------------------------------------------

//-//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//+++++++++++++++++
void DynamicCar::evolutionControl(double dt) {
    if (fabs(car_state.steer_alpha_req) >= car_par.max_steer)
      car_state.steer_alpha_req =
          (car_par.max_steer - 0.00001) *
          (car_state.steer_alpha_req / fabs(car_state.steer_alpha_req));

    car_state.FXf_req = car_state.FXf_act;

    car_state.FXr_req = car_state.FXr_act;

    double Fbrake_f = 0.0;

    double Fbrake_r = 0.0;


    if (culc_par.fl_act_low < 1) {  // For simplifying

      car_state.steer_alpha = car_state.steer_alpha_req;

    } else {
      double steer_change = car_par.maxGradSteer * dt;

      if((culc_par.fl_RunKut > 0)&&(car_state.state == 0))
          steer_change = 0.0;

      if (fabs(car_state.steer_alpha_req - car_state.steer_alpha) <= steer_change)
        car_state.steer_alpha = car_state.steer_alpha_req;
      else {
        double dir_ch = 1.0;

        if (car_state.steer_alpha > car_state.steer_alpha_req) dir_ch = -1.0;

        car_state.steer_alpha += steer_change * dir_ch;
      }

      if (culc_par.fl_act_low > 1) {
        double press_change = car_par.maxGradBrakePressure * dt;

        if (fabs(car_state.B_pressure_req - car_state.B_pressure) <= press_change)
          car_state.B_pressure = car_state.B_pressure_req;
        else {
          double dir_ch = 1.0;

          if (car_state.B_pressure > car_state.B_pressure_req) dir_ch = -1.0;

          car_state.B_pressure += press_change * dir_ch;
        }

        Fbrake_f = car_par.n_brakes * car_state.B_pressure *
                          car_par.kBrake_f / car_par.frontWheelRad;
        Fbrake_r = car_par.n_brakes * car_state.B_pressure *
                          car_par.kBrake_r / car_par.rearWheelRad;

   //     if (fl_reverse < 1) {
   //       Fbrake_f *= -1.0;
  //        Fbrake_r *= -1.0;
  //      }

        if (fl_AccBrkMode > 0) {
          // rear engine

          double Fmax =
              car_par.n_engines * car_par.maxTorqueEngine / car_par.rearWheelRad;

          if (fabs(car_state.FXr_act) > Fmax)
            car_state.FXr_req =
                car_state.FXr_act * Fmax / fabs(car_state.FXr_act);

          double VWheel = fabs(car_state.Vx);

          double Power_req = fabs(car_state.FXr_req) * VWheel;

          if (Power_req > car_par.n_engines * car_par.maxPowerEngine)
            car_state.FXr_req =
                car_state.FXr_req *
                (car_par.n_engines * car_par.maxPowerEngine / Power_req);

          // engine curve
          double OmWheel = VWheel / car_par.rearWheelRad;

          if (OmWheel >= car_par.Om_nullTorque) {
            //          std::cout << "-> FXr_req:" << car_state.FXr_req //
            //                    << "-> OmWheel:" << OmWheel //
            //                    << "-> Om_nullTorque:" << car_par.Om_nullTorque
            //                    //
            //                    << std::endl;
            car_state.FXr_req = 0.0;
        //    std::cout << "-> FXr_req:" << car_state.FXr_req << std::endl;
          } else if (OmWheel > car_par.Om_maxTorque) {
            double F1 = car_par.n_engines * car_par.maxPowerEngine /
                        (car_par.Om_maxTorque * car_par.rearWheelRad);

            Fmax = F1 * (car_par.Om_nullTorque - OmWheel) /
                   (car_par.Om_nullTorque - car_par.Om_maxTorque);

            if (fabs(car_state.FXr_req) > Fmax) {
              if (car_state.FXr_req < 0.0)
                car_state.FXr_req = (-1.0) * Fmax;
              else
                car_state.FXr_req = Fmax;
            }
          }

          // front engine

          Fmax =
              car_par.n_engines * car_par.maxTorqueEngine / car_par.frontWheelRad;

          if (fabs(car_state.FXf_act) > Fmax)
            car_state.FXf_req =
                car_state.FXf_act * Fmax / fabs(car_state.FXf_act);

          VWheel = fabs(car_state.Vx / cos(car_state.steer_alpha));

          Power_req = fabs(car_state.FXf_req) * VWheel;

          if (Power_req > car_par.n_engines * car_par.maxPowerEngine)
            car_state.FXf_req =
                car_state.FXf_req *
                (car_par.n_engines * car_par.maxPowerEngine / Power_req);

          // engine curve
          OmWheel = VWheel / car_par.frontWheelRad;

          if (OmWheel >= car_par.Om_nullTorque)
            car_state.FXf_req = 0.0;
          else if (OmWheel > car_par.Om_maxTorque) {
            double F1 = car_par.n_engines * car_par.maxPowerEngine /
                        (car_par.Om_maxTorque * car_par.frontWheelRad);

            Fmax = F1 * (car_par.Om_nullTorque - OmWheel) /
                   (car_par.Om_nullTorque - car_par.Om_maxTorque);

            if (fabs(car_state.FXf_req) > Fmax) {
              if (car_state.FXf_req < 0.0)
                car_state.FXf_req = (-1.0) * Fmax;
              else
                car_state.FXf_req = Fmax;
            }
          }
        }

    //    car_state.FXf_req += Fbrake_f;
    //    car_state.FXr_req += Fbrake_r;
      }
    }


    double drFrc_f = car_par.rollingResistance*car_par.drive_distrib;
    double drFrc_r = car_par.rollingResistance*(1.0 - car_par.drive_distrib);

    if(culc_par.fl_drag_taking == 0){

        drFrc_f = 0.0;
        drFrc_r = 0.0;


    }


    Fbrake_f += drFrc_f;
    Fbrake_r += drFrc_r;

    if(car_state.state == 0){


       if(fabs(car_state.FXf_req + car_state.FXr_req) - Fbrake_f - Fbrake_r > 1.0){


          if(fl_reverse < 1){

              Fbrake_f *= -1.0;
              Fbrake_r *= -1.0;
          }

          car_state.FXf_req += Fbrake_f;
          car_state.FXr_req += Fbrake_r;

          car_state.state = 1;
       }


    }
    else{




            if(culc_par.fl_Long_Circ < 2){

               car_state.OmWr = car_state.Vx/car_par.rearWheelRad;

               double Vxf = car_state.Vx;
               double Vyf = car_state.Vy + car_state.Om * car_par.a;

               double cs = cos(car_state.steer_alpha);
               double sn = sin(car_state.steer_alpha);

               double Vxw = Vxf * cs + Vyf * sn;
               double Vyw = Vyf * cs - Vxf * sn;

               car_state.OmWf = Vxw/car_par.frontWheelRad;

            }

            double fct_br_f = 1.0;

            double fct_br_r = 1.0;

            if(fl_reverse == 0){


                fct_br_f = -1.0;
                fct_br_r = -1.0;

                if(car_state.OmWf < 0.0)
                 fct_br_f = 1.0;


                if(car_state.OmWr < 0.0)
                 fct_br_r = 1.0;

            }
            else{

                if(car_state.OmWf > 0.0)
                 fct_br_f = -1.0;


                if(car_state.OmWr > 0.0)
                 fct_br_r = -1.0;

            }

            Fbrake_f *= fct_br_f;
            Fbrake_r *= fct_br_r;

            car_state.FXf_req += Fbrake_f;
            car_state.FXr_req += Fbrake_r;


    }



}

//-----------------------------------------------------------------------------------------

//-//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//+++++++++++++++++

void DynamicCar::newLongForce() {
  // verical force distribution:

  getVerticalLoads();

  // tyre force distribution:

  // REAR:

  double FXBound_r = car_par.mu * car_state.Nr;

  if (culc_par.fl_Long_Circ < 0) {
    if (fabs(car_state.FYr) >= FXBound_r)
      FXBound_r = 0.0;
    else
      FXBound_r = sqrt(FXBound_r * FXBound_r - car_state.FYr * car_state.FYr);
  }

  if (fabs(car_state.FXr_req) < 0.001)
    car_state.FXr = 0.0;
  else if(culc_par.fl_Long_Circ <= 0){

    if (fabs(car_state.FXr_req) > FXBound_r) {
      car_state.FXr = FXBound_r;

      if (car_state.FXr_req < 0.0) car_state.FXr = (-1.0) * car_state.FXr;

    } else
      car_state.FXr = car_state.FXr_req;
  }
  else car_state.FXr = car_state.FXr_req;

  // FORWARD:

  double FXBound_f = car_par.mu * car_state.Nf;

  if (culc_par.fl_Long_Circ < 0) {
    if (fabs(car_state.FYf) >= FXBound_f)
      FXBound_f = 0.0;
    else
      FXBound_f = sqrt(FXBound_f * FXBound_f - car_state.FYf * car_state.FYf);
  }

  if (fabs(car_state.FXf_req) < 0.001)
    car_state.FXf = 0.0;
  else if(culc_par.fl_Long_Circ <= 0){
    if (fabs(car_state.FXf_req) > FXBound_f) {
      car_state.FXf = FXBound_f;

      if (car_state.FXf_req < 0.0) car_state.FXf = (-1.0) * car_state.FXf;
    } else
      car_state.FXf = car_state.FXf_req;
  }
  else car_state.FXf = car_state.FXf_req;

}
//--------------------------------------------------------------------------------------------------------------------

//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::newLatForce() {
  getLatSlip();
  getVerticalLoads();

  // FORWARD

  car_state.FYf_req =
      brushModel(car_state.slipAng_f, car_state.Nf, car_par.Cy_f);

  if (culc_par.fl_Long_Circ < 0) {
    double FBound = car_par.mu * car_state.Nf;

    if (fabs(car_state.FXf) >= FBound)
      FBound = 0.0;
    else
      FBound = sqrt(FBound * FBound - car_state.FXf * car_state.FXf);

    if (fabs(car_state.FYf_req) > FBound) {
      car_state.FYf = FBound;

      if (car_state.FYf_req < 0.0) car_state.FYf = (-1.0) * FBound;
    } else
      car_state.FYf = car_state.FYf_req;
  } else
    car_state.FYf = car_state.FYf_req;

  // REAR

  car_state.FYr_req =
      brushModel(car_state.slipAng_r, car_state.Nr, car_par.Cy_r);

  if (culc_par.fl_Long_Circ < 0) {
    double FBound = car_par.mu * car_state.Nr;

    if (fabs(car_state.FXr) >= FBound)
      FBound = 0.0;
    else
      FBound = sqrt(FBound * FBound - car_state.FXr * car_state.FXr);

    if (fabs(car_state.FYr_req) > FBound) {
      car_state.FYr = FBound;

      if (car_state.FYr_req < 0.0) car_state.FYr = (-1.0) * FBound;
    } else
      car_state.FYr = car_state.FYr_req;
  } else
    car_state.FYr = car_state.FYr_req;

}

//--------------------------------------------------------------------------------------------------------------------

//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::getLatSlip() {
  // Rear:

  double Vxr = car_state.Vx;
  double Vyr = car_state.Vy - car_state.Om * car_par.b;

  if (Vxr < 0.0) Vxr = (-1.0) * Vxr;

  car_state.slipAng_r = atan2(Vyr, Vxr);

 // if (Vxr < 0.001) car_state.slipAng_r = 0.0;

  double Vxf = car_state.Vx;
  double Vyf = car_state.Vy + car_state.Om * car_par.a;

  double cs = cos(car_state.steer_alpha);
  double sn = sin(car_state.steer_alpha);

  double Vxw = Vxf * cs + Vyf * sn;
  double Vyw = Vyf * cs - Vxf * sn;

  if (Vxw < 0.0) Vxw = (-1.0) * Vxw;

  car_state.slipAng_f = atan2(Vyw, Vxw);

 // if (Vxw < 0.001) car_state.slipAng_f = 0.0;
}

//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
double DynamicCar::brushModel_solver(double FY, int fl_forwardWheel,
                                     int& fl_OK) {
  // TO DO: It should be changed to lookup-table;

  double Fslip = fabs(FY);

  double Ctyre = car_par.Cy_f;

  double Fz = car_par.mass * 9.89 / car_par.L;

  if (fl_forwardWheel > 0) {
    Fz = Fz * car_par.b;

  } else {
    Ctyre = car_par.Cy_r;
    Fz = Fz * car_par.a;
  }

  fl_OK = 1.0;

  double alpha_slide = car_par.mu * ((3.0 * Fz) / Ctyre);

  double slip = 0.0;

  if (Fslip >= car_par.mu * Fz) {
    slip = alpha_slide;

    if (FY > 0) slip = slip * (-1.0);

    return slip;
  }

  if (Fslip <= Ctyre * 0.0001)
    slip = Fslip / Ctyre;
  else {
    slip = alpha_slide * 0.5;

    int iter = 0;

    do {
      iter++;

      fl_OK = iter;

      double F = -brushModel(slip, Fz, Ctyre) - Fslip;

      if (fabs(F) < 1.0) break;

      if (iter > 8) {
        fl_OK = 0;
        break;
      }

      if (true)
        if (slip > 0.98 * alpha_slide) {
          if (F < 0.0) {
            double F1 =
                -brushModel(slip + 0.5 * (alpha_slide - slip), Fz, Ctyre) -
                Fslip;

            if (fabs(F1) < fabs(F)) slip += 0.5 * (alpha_slide - slip);

            break;

          } else {
            slip = 0.98 * alpha_slide;
          }
        }

      double slip_rel = slip / alpha_slide;

      double dF = Ctyre * (1.0 - 2.0 * slip_rel + slip_rel * slip_rel);

      slip += -F / dF;

      if (slip < 0.0) slip = 0.0;

      if (slip >= alpha_slide) slip = 0.99 * alpha_slide;

    } while (true);
  }

  if (FY > 0) slip = slip * (-1.0);

  return slip;
}
//-------------------------------------------------------------------------------------------------------------------

//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
double DynamicCar::brushModel(double slip_angle, double Fz, double Ctyre) {
  double Fslip = 0.0;

  double alpha_slide = car_par.mu * ((3.0 * Fz) / Ctyre);

  if (fabs(slip_angle) >= alpha_slide) {
    Fslip = car_par.mu * Fz;

    if (slip_angle > 0.0) Fslip = Fslip * (-1.0);

  } else {
    double tal = tan(slip_angle);
    double slip_rel = fabs(tal) / alpha_slide;

    Fslip = -Ctyre * tal * (1.0 - slip_rel + (1.0 / 3.0) * slip_rel * slip_rel);
  }

  return Fslip;
}
//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
double DynamicCar::brushModel_combined(double slip_angle, double Fz) {
  double Fslip = 0.0;

  double alpha_slide = car_par.mu * (3.0 * Fz);

  if (fabs(slip_angle) >= alpha_slide) {
    Fslip = car_par.mu * Fz;

  } else {
    double slip_rel = fabs(slip_angle) / alpha_slide;

    Fslip = slip_angle * (1.0 - slip_rel + (1.0 / 3.0) * slip_rel * slip_rel);
  }

  return Fslip;
}
//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::combinedSlip() {
  if (culc_par.fl_Long_Circ < 2) {
    newLatForce();   //
    newLongForce();  // Here Longitude and Lateral tyre forces being evaluated;
    // these functions use (or don't use) cyrcle of tyre force (rough model).

    if(culc_par.fl_Long_Circ == 1){

        double f_r = sqrt(car_state.FXr*car_state.FXr + car_state.FYr*car_state.FYr);

        double fmax_r = car_state.Nr*car_par.mu;

        if(f_r > fmax_r){

            double df = fmax_r/f_r;

            car_state.FXr *= df;
            car_state.FYr *= df;
        }

        double f_f = sqrt(car_state.FXf*car_state.FXf + car_state.FYf*car_state.FYf);
        double fmax_f = car_state.Nf*car_par.mu;

        if(f_f > fmax_f){

             double df = fmax_f/f_f;

             car_state.FXf *= df;
             car_state.FYf *= df;
        }
    }

  } else {
    getVerticalLoads();

    double Vxr = car_state.Vx;
    double WVr = car_state.OmWr * car_par.rearWheelRad;
    double Vyr = car_state.Vy - car_state.Om * car_par.b;

    if (culc_par.fl_Long_Circ < 3)
      if ((car_par.drive_distrib > 0.99) && (fl_AccBrkMode > 0)) {
        WVr = Vxr;
        car_state.OmWr = Vxr / car_par.rearWheelRad;
      }

    double DWr = WVr - Vxr;

    double normK_r = fabs(WVr) + 0.0001;

    car_state.long_slip_r = DWr / normK_r;

    if (Vxr < 0.0) Vxr = (-1.0) * Vxr;

    car_state.slipAng_r = atan2(Vyr, Vxr);

    double sig_xr = car_par.Cx_r * car_state.long_slip_r;
    double sig_yr = -car_par.Cy_r * Vyr / normK_r;

    double gama_r = sqrt(sig_xr * sig_xr + sig_yr * sig_yr);

    double Fslip_r = brushModel_combined(gama_r, car_state.Nr);

    car_state.FXr = (sig_xr / (gama_r + 0.0001)) * Fslip_r;
    car_state.FYr = (sig_yr / (gama_r + 0.0001)) * Fslip_r;

    double Vxf = car_state.Vx;
    double Vyf = car_state.Vy + car_state.Om * car_par.a;

    double cs = cos(car_state.steer_alpha);
    double sn = sin(car_state.steer_alpha);

    double Vxw = Vxf * cs + Vyf * sn;
    double Vyw = Vyf * cs - Vxf * sn;

    double WVf = car_state.OmWf * car_par.frontWheelRad;

    if (culc_par.fl_Long_Circ < 3)
      if ((car_par.drive_distrib < 0.01) && (fl_AccBrkMode > 0)) {
        WVf = Vxw;
        car_state.OmWf = Vxw / car_par.frontWheelRad;
      }

    double DWf = WVf - Vxw;

    double normK_f = fabs(WVf) + 0.0001;

    car_state.long_slip_f = DWf / normK_f;

    if (Vxw < 0.0) Vxw = (-1.0) * Vxw;

    car_state.slipAng_f = atan2(Vyw, Vxw);

    double sig_xf = car_par.Cx_f * car_state.long_slip_f;
    double sig_yf = -car_par.Cy_f * Vyw / normK_f;

    double gama_f = sqrt(sig_xf * sig_xf + sig_yf * sig_yf);

    double Fslip_f = brushModel_combined(gama_f, car_state.Nf);

    car_state.FXf = (sig_xf / (gama_f + 0.0001)) * Fslip_f;
    car_state.FYf = (sig_yf / (gama_f + 0.0001)) * Fslip_f;
  }


}

//---------------------------------------------------------------------

//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////

void DynamicCar::getAccBrkMode() {
  fl_AccBrkMode = 1.0;  // acceleration mode; -1.0 - braking mode

  if (fabs(car_state.accel) < 0.00001) return;

  double fl = 1000.0;

  if (fl_reverse > 0) fl = -1000.0;

  if (fl * car_state.accel < 0.0) fl_AccBrkMode = -1.0;
}
//---------------------------------------------------------------------
//-////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-/////////////////////////////////////
void DynamicCar::addToMPClog(double dtk) {
  combinedSlip();

  //   fileMPClog << timeMPClog
  //              << "     "
  //              << car_state.steer_alpha << "  "
  //              << car_state.FXf_req << "  " << car_state.FXr_req << "  "
  //              << car_state.Vx << "  " << car_state.Vy << "  " <<
  //              car_state.Om << "  " << car_state.x << "  " << car_state.y <<
  //              "  "<< car_state.psi << "  "
  //              << car_state.OmWf << "  " << car_state.OmWr << "  " <<
  //              car_state.dVxdt << "  " << car_state.dVydt << "  " <<
  //              car_state.dOmdt << "  "
  //              << car_state.Vx*cos(car_state.psi) -
  //              car_state.Vy*sin(car_state.psi) << "  "
  //              << car_state.Vx*sin(car_state.psi) +
  //              car_state.Vy*cos(car_state.psi) << "  "
  //              << car_state.Om << "  "
  //              << car_state.dOmWfdt << "  " << car_state.dOmWrdt << "  "
  //              << std::endl;

  timeMPClog += dtk;
}
//---------------------------------------------------------------------
//-////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-/////////////////////////////////////
void DynamicCar::addToLog(double dt) {
  //------------------------------------------
  in_time += dt;
  d_time += dt;

  if(false){
  if (d_time >= 0.01) {
    double FY = (1.0 / car_par.L) * car_par.mass * car_state.Vx * car_state.Om;

    double FYr_st = car_par.a * FY;
    double FYf_st = car_par.b * FY;

    int flOKr = 1;
    double al_r = brushModel_solver(FYr_st, 0, flOKr);
    if (flOKr < 1) al_r = 0.0;

    int flOKf = 1;
    double al_f = brushModel_solver(FYf_st, 1, flOKf);
    if (flOKf < 1) al_f = 0.0;

    double Vdes = car_state.Vdes;

    if (fl_reverse > 0) Vdes = -Vdes;

   // combinedSlip();

#if 0
    // fileDyn
    LOG(INFO)
        << in_time << "      "
        << (180.0 / M_PI) * car_state.Om
        //<< "   " <<   (180.0/M_PI)*(tan(car_state.steer_alpha
        //+car_state.slipAng_f) -
        // tan(car_state.slipAng_r))*car_state.Vx/car_par.L
        << "   "
        << (180.0 / M_PI) * (tan(car_state.steer_alpha + al_f) - tan(al_r)) *
               car_state.Vx / car_par.L
        << "   "
        << (180.0 / M_PI) * tan(car_state.steer_alpha) * car_state.Vx /
               car_par.L
        << "      " << car_state.slipAng_r * (180.0 / M_PI) << "   "
        << al_r * (180.0 / M_PI) << "      "
        << car_state.slipAng_f * (180.0 / M_PI) << "   "
        << al_f * (180.0 / M_PI) << "      "
        << car_state.steer_alpha * (180.0 / M_PI) << "   "
        << car_state.steer_alpha_req * (180.0 / M_PI) << "      "
        << car_state.Vx << "   " << Vdes << "     " << car_state.x << "   "
        << car_state.y << "   " << car_state.psi << "      " << car_state.Vy
        << "   "
        << car_state.Vx *
               (car_par.b * tan(car_state.steer_alpha) / car_par.L + tan(al_r))
        << "      "
        << (180.0 / M_PI) * atan(car_state.Vy / (car_state.Vx + 0.00001))
        << "   "
        << (180.0 / M_PI) *
               atan(car_state.Vx *
                    (car_par.b * tan(car_state.steer_alpha) / car_par.L +
                     tan(al_r)) /
                    (car_state.Vx + 0.00001))
        << "      " << car_state.FXr << "  " << car_state.FYr << " "
        << fabs(car_state.Nr * car_par.mu) -
               sqrt(car_state.FXr * car_state.FXr +
                    car_state.FYr * car_state.FYr)
        << "      " << car_state.FXf << "  " << car_state.FYf << " "
        << fabs(car_state.Nf * car_par.mu) -
               sqrt(car_state.FXf * car_state.FXf +
                    car_state.FYf * car_state.FYf)
        << "      " << car_state.state << "      " << car_state.B_pressure
        << "      " << car_state.long_slip_f * (180.0 / M_PI) << "  "
        << car_state.long_slip_r * (180.0 / M_PI) << "      " << car_state.dVxdt
        << "       "
        << car_state.Om * car_state.Vy +
               (car_state.FXf * cos(car_state.steer_alpha) -
                car_state.FYf * sin(car_state.steer_alpha) + car_state.FXr) /
                   car_par.mass -
               car_state.Fdrag / car_par.mass
        << "     " << car_state.accel - (car_state.Fdrag_PM / car_par.mass)
        << std::endl;
#endif

    d_time = 0.0;

    //       std::cout<< "intime = " << in_time << std::endl;
  }
  }
}
//---------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////

void DynamicCar::dynamicNav_ev(double steer, double accel_des, double dt,
                               int reverse_fl, double speed_des) {
  // +++   Here are passed the required control for actuators; evolution of
  // actuators is defined in evolutionControl()

  car_state.steer_alpha_req = steer;
  car_state.accel =
      accel_des;  // with sign corresponding car system of coordinate

  fl_reverse = reverse_fl;
  car_state.Vdes = speed_des;  // For Fdrag_PM  being evaluated

  getAccBrkMode();  // getting choice of fl_AccBrkMode

  getDrag_PM();  // correction of car_state.accel to take into account FDrag_PM
                 // (evaluation of drag force for PM is located here !);

  newFXreq();  // distribution of required forces to engines (forces being
               // distributed as requested values in corresponding to drive
               // type)

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  double dtk = dt / double(culc_par.nsteps);

  int nsteps = 1;

  do {
    if (fl_MPClog > 0) addToMPClog(dtk);

    // integrNavkinematictest(dtk);
    integrNav(dtk);

    // t = t + dtk;

    nsteps++;

  } while (nsteps <= culc_par.nsteps);

  if (fl_fileDyn > 0) addToLog(dt);

  car_state.callcount += 1;
}

//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::analize_Fdrag(double& dVxdt) {
  if (car_state.state == 0) {
    if (fl_reverse > 0) {
      if (dVxdt > 0) dVxdt = 0;

    } else {
      if (dVxdt < 0) dVxdt = 0;
    }
  }
}

//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::integrNav(double dt){

    //dVxdt = Om*Vy + (FXr + FXf - Fdrag)/m;
    //dVydt = -Om*Vx + (FYf*cos(steer) + FXf*sin(steer) + FYr)/m;
    //dOmdt = (-FYr*b + (FYf*cos(steer) +FXf*sin(steer))*a)/mI;
    //dpsidt = Om;
    //dxdt = Vx*cos(psi)-Vy*sin(psi);
    //dydt = Vx*sin(psi)+Vy*cos(psi);


 //   std::cout << "Vx = " << car_state.Vx << "    =====================================" << std::endl;

    if((culc_par.fl_RunKut < 1)||(car_state.state == 0))
        evolutionControl(dt);


    if(car_state.state == 0)
        return;



    // in the case of Run-Kut the actuators have initial state on this step;

    combinedSlip();
    getDrag();

    //std::cout << "Vy: " << car_state.Vy << "\n";


    double kx1 = (car_state.Vx*cos(car_state.psi) - car_state.Vy*sin(car_state.psi));
    double ky1 = (car_state.Vx*sin(car_state.psi) + car_state.Vy*cos(car_state.psi));
    double kpsi1 = car_state.Om;
    double kVx1 = (car_state.Om*car_state.Vy + (car_state.FXr + car_state.FXf*cos(car_state.steer_alpha) - car_state.FYf*sin(car_state.steer_alpha)
                                                                                                            -car_state.Fdrag)/car_par.mass);
    double kVy1 = (-car_state.Om*car_state.Vx + (car_state.FYr + car_state.FXf*sin(car_state.steer_alpha) +
                                                   car_state.FYf*cos(car_state.steer_alpha) )/car_par.mass);
    double kOm1 = (-car_state.FYr*car_par.b + car_state.FYf*cos(car_state.steer_alpha)*car_par.a +
                      car_state.FXf*sin(car_state.steer_alpha)*car_par.a)/car_par.moment_inertia;

    double kOmWf1 = car_par.frontWheelRad*(car_state.FXf_req - car_state.FXf)/car_par.fWIn;
    double kOmWr1 = car_par.rearWheelRad*(car_state.FXr_req - car_state.FXr)/car_par.rWIn;


    double x0 = car_state.x;
    double y0 = car_state.y;
    double psi0 = car_state.psi;
    double Vx0 = car_state.Vx;
    double Vy0 = car_state.Vy;
    double Om0 = car_state.Om;

    double OmWf0 = car_state.OmWf;
    double OmWr0 = car_state.OmWr;


    if(culc_par.fl_RunKut > 0)
    {


        car_state.x = x0 + 0.5*kx1*dt;
        car_state.y = y0 + 0.5*ky1*dt;
        car_state.psi = psi0 + 0.5*kpsi1*dt;
        car_state.Vx = Vx0 + 0.5*kVx1*dt;
        car_state.Vy = Vy0 + 0.5*kVy1*dt;
        car_state.Om = Om0 + 0.5*kOm1*dt;

        if(culc_par.fl_Long_Circ > 1){

            car_state.OmWf = OmWf0 + 0.5*kOmWf1*dt;
            car_state.OmWr = OmWr0 + 0.5*kOmWr1*dt;

        }

        evolutionControl(0.5*dt);

        combinedSlip();
        getDrag();

        double kx2 = (car_state.Vx*cos(car_state.psi) - car_state.Vy*sin(car_state.psi));
        double ky2 = (car_state.Vx*sin(car_state.psi) + car_state.Vy*cos(car_state.psi));
        double kpsi2 = car_state.Om;
        double kVx2 = (car_state.Om*car_state.Vy + (car_state.FXr + car_state.FXf*cos(car_state.steer_alpha) - car_state.FYf*sin(car_state.steer_alpha)
                                                    - car_state.Fdrag)/car_par.mass);

        double kVy2 = (-car_state.Om*car_state.Vx + (car_state.FYr + car_state.FXf*sin(car_state.steer_alpha) +
                                                   car_state.FYf*cos(car_state.steer_alpha) )/car_par.mass);
        double kOm2 = (-car_state.FYr*car_par.b + car_state.FYf*cos(car_state.steer_alpha)*car_par.a +
                      car_state.FXf*sin(car_state.steer_alpha)*car_par.a)/car_par.moment_inertia;


        double kOmWf2 = car_par.frontWheelRad*(car_state.FXf_req - car_state.FXf)/car_par.fWIn;
        double kOmWr2 = car_par.rearWheelRad*(car_state.FXr_req - car_state.FXr)/car_par.rWIn;



        car_state.x = x0 + 0.5*kx2*dt;
        car_state.y = y0 + 0.5*ky2*dt;
        car_state.psi = psi0 + 0.5*kpsi2*dt;
        car_state.Vx = Vx0 + 0.5*kVx2*dt;
        car_state.Vy = Vy0 + 0.5*kVy2*dt;
        car_state.Om = Om0 + 0.5*kOm2*dt;

        if(culc_par.fl_Long_Circ > 1){

            car_state.OmWf = OmWf0 + 0.5*kOmWf2*dt;
            car_state.OmWr = OmWr0 + 0.5*kOmWr2*dt;
        }

        // actuators have the same state (t+0.5*dt)

        combinedSlip();
        getDrag();

        double kx3 = (car_state.Vx*cos(car_state.psi) - car_state.Vy*sin(car_state.psi));
        double ky3 = (car_state.Vx*sin(car_state.psi) + car_state.Vy*cos(car_state.psi));
        double kpsi3 = car_state.Om;
        double kVx3 = (car_state.Om*car_state.Vy + (car_state.FXr + car_state.FXf*cos(car_state.steer_alpha) - car_state.FYf*sin(car_state.steer_alpha)
                                                                        -car_state.Fdrag)/car_par.mass);

        double kVy3 = (-car_state.Om*car_state.Vx + (car_state.FYr + car_state.FXf*sin(car_state.steer_alpha) +
                                                   car_state.FYf*cos(car_state.steer_alpha) )/car_par.mass);
        double kOm3 = (-car_state.FYr*car_par.b + car_state.FYf*cos(car_state.steer_alpha)*car_par.a +
                      car_state.FXf*sin(car_state.steer_alpha)*car_par.a)/car_par.moment_inertia;

        double kOmWf3 = car_par.frontWheelRad*(car_state.FXf_req - car_state.FXf)/car_par.fWIn;
        double kOmWr3 = car_par.rearWheelRad*(car_state.FXr_req - car_state.FXr)/car_par.rWIn;


        car_state.x = x0 + kx3*dt;
        car_state.y = y0 + ky3*dt;
        car_state.psi = psi0 + kpsi3*dt;
        car_state.Vx = Vx0 + kVx3*dt;
        car_state.Vy = Vy0 + kVy3*dt;
        car_state.Om = Om0 + kOm3*dt;

        if(culc_par.fl_Long_Circ > 1){

            car_state.OmWf = OmWf0 + kOmWf3*dt;
            car_state.OmWr = OmWr0 + kOmWr3*dt;
        }


        evolutionControl(0.5*dt); // actuators have changed in corresponding to changing time from t to t + dt
        combinedSlip();
        getDrag();

        double kx4 = (car_state.Vx*cos(car_state.psi) - car_state.Vy*sin(car_state.psi));
        double ky4 = (car_state.Vx*sin(car_state.psi) + car_state.Vy*cos(car_state.psi));
        double kpsi4 = car_state.Om;
        double kVx4 = (car_state.Om*car_state.Vy + (car_state.FXr + car_state.FXf*cos(car_state.steer_alpha) - car_state.FYf*sin(car_state.steer_alpha)
                                                    - car_state.Fdrag)/car_par.mass);

        double kVy4 = (-car_state.Om*car_state.Vx + (car_state.FYr + car_state.FXf*sin(car_state.steer_alpha) +
                                                   car_state.FYf*cos(car_state.steer_alpha) )/car_par.mass);
        double kOm4 = (-car_state.FYr*car_par.b + car_state.FYf*cos(car_state.steer_alpha)*car_par.a +
                      car_state.FXf*sin(car_state.steer_alpha)*car_par.a)/car_par.moment_inertia;

        double kOmWf4 = car_par.frontWheelRad*(car_state.FXf_req - car_state.FXf)/car_par.fWIn;
        double kOmWr4 = car_par.rearWheelRad*(car_state.FXr_req - car_state.FXr)/car_par.rWIn;


        // Runge modifications:

        kx1 = (kx1 + 2*kx2 + 2*kx3 + kx4)/6.0;
        ky1 = (ky1 + 2*ky2 + 2*ky3 + ky4)/6.0;
        kpsi1 = (kpsi1 + 2*kpsi2 + 2*kpsi3 + kpsi4)/6.0;
        kVx1 = (kVx1 + 2*kVx2 + 2*kVx3 + kVx4)/6.0;
        kVy1 = (kVy1 + 2*kVy2 + 2*kVy3 + kVy4)/6.0;
        kOm1 = (kOm1 + 2*kOm2 + 2*kOm3 + kOm4)/6.0;

        kOmWf1 = (kOmWf1 + 2*kOmWf2 + 2*kOmWf3 + kOmWf4)/6.0;
        kOmWr1 = (kOmWr1 + 2*kOmWr2 + 2*kOmWr3 + kOmWr4)/6.0;

    }
    else{

        car_state.dVxdt = kVx1;
        car_state.dOmWrdt = kOmWr1;

        double dti = analize_dt(dt);

        while(dt > dti){

            std::cout << "ky1 : " << ky1 << "\n";

            car_state.x = x0 + kx1*dti;
            car_state.y = y0 + ky1*dti;
            car_state.psi = psi0 + kpsi1*dti;
            car_state.Vx = Vx0 + kVx1*dti;
            car_state.Vy = Vy0 + kVy1*dti;
            car_state.Om = Om0 + kOm1*dti;

            if(culc_par.fl_Long_Circ > 1){
                car_state.OmWf = OmWf0 + kOmWf1*dti;
                car_state.OmWr = OmWr0 + kOmWr1*dti;
            }

            if(dt > dti)
                dt = dt - dti;
            else
                dti = dt;

            combinedSlip();
            getDrag();

            kx1 = (car_state.Vx*cos(car_state.psi) - car_state.Vy*sin(car_state.psi));
            ky1 = (car_state.Vx*sin(car_state.psi) + car_state.Vy*cos(car_state.psi));
            kpsi1 = car_state.Om;
            kVx1 = (car_state.Om*car_state.Vy + (car_state.FXr + car_state.FXf*cos(car_state.steer_alpha) - car_state.FYf*sin(car_state.steer_alpha)
                                                        - car_state.Fdrag)/car_par.mass);

            kVy1 = (-car_state.Om*car_state.Vx + (car_state.FYr + car_state.FXf*sin(car_state.steer_alpha) +
                                                           car_state.FYf*cos(car_state.steer_alpha) )/car_par.mass);
            kOm1 = (-car_state.FYr*car_par.b + car_state.FYf*cos(car_state.steer_alpha)*car_par.a +
                              car_state.FXf*sin(car_state.steer_alpha)*car_par.a)/car_par.moment_inertia;

            kOmWf1 = car_par.frontWheelRad*(car_state.FXf_req - car_state.FXf)/car_par.fWIn;
            kOmWr1 = car_par.rearWheelRad*(car_state.FXr_req - car_state.FXr)/car_par.rWIn;


            x0 = car_state.x;
            y0 = car_state.y;
            psi0 = car_state.psi;
            Vx0 = car_state.Vx;
            Vy0 = car_state.Vy;
            Om0 = car_state.Om;

            OmWf0 = car_state.OmWf;
            OmWr0 = car_state.OmWr;
        }

    }




    car_state.x = x0 + kx1*dt;
    car_state.y = y0 + ky1*dt;
    car_state.psi = psi0 + kpsi1*dt;
    car_state.Vx = Vx0 + kVx1*dt;
    car_state.Vy = Vy0 + kVy1*dt;
    car_state.Om = Om0 + kOm1*dt;

    if(culc_par.fl_Long_Circ > 1){
        car_state.OmWf = OmWf0 + kOmWf1*dt;
        car_state.OmWr = OmWr0 + kOmWr1*dt;
    }

    car_state.dVxdt = kVx1;
    car_state.dVydt = kVy1;
    car_state.dOmdt = kOm1;
    car_state.dOmWfdt = kOmWf1;
    car_state.dOmWrdt = kOmWr1;



    if(fl_AccBrkMode < 0){

            double Vvh = sqrt(car_state.Vx*car_state.Vx + car_state.Vy*car_state.Vy);

            if(Vvh < 0.01)
            {


                car_state.Vx = 0.0;
                car_state.Vy = 0.0;
                car_state.Om = 0.0;

                car_state.OmWf = 0.0;
                car_state.OmWr = 0.0;

                car_state.long_slip_r = 0.0;
                car_state.long_slip_f = 0.0;

                car_state.slipAng_r = 0.0;
                car_state.slipAng_f = 0.0;

                car_state.dVxdt = 0.0;
                car_state.dVydt = 0.0;
                car_state.dOmdt = 0.0;
                car_state.dOmWfdt = 0.0;
                car_state.dOmWrdt = 0.0;

                car_state.state = 0;


            }



    }


    if(culc_par.fl_Long_Circ > 1)
       combinedSlip();

    nav_to_cog(1);



}
//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
double DynamicCar::analize_dt(double t){



    double dt = t;

    if(culc_par.fl_RunKut >= 0)
      return dt;

    if(fl_first_call == 1){

        fl_first_call =0;

        return dt = 0.0000001;
    }


    if(culc_par.fl_RunKut < -1){

        if(car_par.drive_distrib > 0.01)
            return dt;

        if(car_state.OmWr < 0.0)
            return dt;

        if(car_state.Vx < 0.0)
            return dt;

        double VW = car_par.rearWheelRad*car_state.OmWr;
        double AW = car_par.rearWheelRad*car_state.dOmWrdt;

        double f = (-VW*car_state.dVxdt + AW*car_state.Vx + 0.001*(AW - car_state.dVxdt))/((VW +0.001)*(VW + 0.001));

        if(f <= 1.0)
            return dt;

        if(f >= 10000.0)
            dt = 0.000001;
        else
            dt = 0.01/f;

        dt = 0.0000001;
    }
    else{

        double Vxr = car_state.Vx;
        double WVr = car_state.OmWr*car_par.rearWheelRad;
        double Vyr = car_state.Vy - car_state.Om*car_par.b;

        double DWr = WVr - Vxr;

        double normK_r = fabs(WVr) + 0.001;

        double long_slip_r = fabs(DWr/normK_r);


        if(culc_par.fl_Long_Circ < 3)
            if((car_par.drive_distrib > 0.99)&&(fl_AccBrkMode > 0)){

               long_slip_r = 0.0;
            }

        if(Vxr < 0.0)
            Vxr = (-1.0)*Vxr;

        double slipAng_r = fabs(atan2(Vyr,Vxr+0.001));


        double Vxf = car_state.Vx;
        double Vyf = car_state.Vy + car_state.Om*car_par.a;

        double cs = cos(car_state.steer_alpha);
        double sn = sin(car_state.steer_alpha);

        double Vxw = Vxf*cs + Vyf*sn;
        double Vyw = Vyf*cs - Vxf*sn;



        double WVf = car_state.OmWf*car_par.frontWheelRad;



        double DWf = WVf - Vxw;

        double normK_f = fabs(WVf) + 0.001;

        double long_slip_f = fabs(DWf/normK_f);

        if(culc_par.fl_Long_Circ < 3)
            if((car_par.drive_distrib < 0.01)&&(fl_AccBrkMode > 0)){

                long_slip_f = 0.0;
            }


        if(Vxw < 0.0)
            Vxw = (-1.0)*Vxw;

        double slipAng_f = fabs(atan2(Vyw,Vxw + 0.001));

        double lim_angle = 2.0*(M_PI/180.0);

        if(((long_slip_f > lim_angle)||(slipAng_f > lim_angle))||((long_slip_r > lim_angle)||(slipAng_r > lim_angle)))
            dt = 0.0000001;
        else if(fabs(car_state.Vx) < 2.0)
            dt = 0.0000001;


    }

  //  std::cout << " ----------------------- dt correction ---------------------------" << std::endl;

    return dt;


}
//-------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::integrNavkinematictest(double dt) {
  // dVxdt = ax;
  // Vy = 0
  // Om = Vx*tan(steer_alpha)/L;
  // dpsidt = Om;
  // dxdt = Vx*cos(psi);
  // dydt = Vx*sin(psi);

  if (culc_par.fl_RunKut < 1) evolutionControl(dt);

  combinedSlip();

  car_state.Om = car_state.Vx * tan(car_state.steer_alpha) / car_par.L;
  car_state.Vy = car_state.Om * car_par.b;
  double kx1 =
      (car_state.Vx * cos(car_state.psi) - car_state.Vy * sin(car_state.psi));
  double ky1 =
      (car_state.Vx * sin(car_state.psi) + car_state.Vy * cos(car_state.psi));
  double kpsi1 = car_state.Om;
  double kVx1 = car_state.accel;

  double x0 = car_state.x;
  double y0 = car_state.y;
  double psi0 = car_state.psi;
  double Vx0 = car_state.Vx;

  if (culc_par.fl_RunKut > 0) {
    car_state.x = x0 + 0.5 * kx1 * dt;
    car_state.y = y0 + 0.5 * ky1 * dt;
    car_state.psi = psi0 + 0.5 * kpsi1 * dt;
    car_state.Vx = Vx0 + 0.5 * kVx1 * dt;

    car_state.dVxdt = car_state.accel;

    evolutionControl(0.5 * dt);

    car_state.Om = car_state.Vx * tan(car_state.steer_alpha) / car_par.L;
    car_state.Vy = car_state.Om * car_par.b;
    double kx2 =
        (car_state.Vx * cos(car_state.psi) - car_state.Vy * sin(car_state.psi));
    double ky2 =
        (car_state.Vx * sin(car_state.psi) + car_state.Vy * cos(car_state.psi));
    double kpsi2 = car_state.Om;
    double kVx2 = car_state.accel;

    car_state.x = x0 + 0.5 * kx2 * dt;
    car_state.y = y0 + 0.5 * ky2 * dt;
    car_state.psi = psi0 + 0.5 * kpsi2 * dt;
    car_state.Vx = Vx0 + 0.5 * kVx2 * dt;

    car_state.dVxdt = car_state.accel;

    // actuators have the same state (t+0.5*dt)

    car_state.Om = car_state.Vx * tan(car_state.steer_alpha) / car_par.L;
    car_state.Vy = car_state.Om * car_par.b;
    double kx3 =
        (car_state.Vx * cos(car_state.psi) - car_state.Vy * sin(car_state.psi));
    double ky3 =
        (car_state.Vx * sin(car_state.psi) + car_state.Vy * cos(car_state.psi));
    double kpsi3 = car_state.Om;
    double kVx3 = car_state.accel;

    car_state.x = x0 + 0.5 * kx3 * dt;
    car_state.y = y0 + 0.5 * ky3 * dt;
    car_state.psi = psi0 + 0.5 * kpsi3 * dt;
    car_state.Vx = Vx0 + 0.5 * kVx3 * dt;

    car_state.dVxdt = car_state.accel;

    evolutionControl(0.5 * dt);  // actuators have changed in corresponding to
                                 // changing time from t to t + dt
    combinedSlip();

    car_state.Om = car_state.Vx * tan(car_state.steer_alpha) / car_par.L;
    car_state.Vy = car_state.Om * car_par.b;
    double kx4 =
        (car_state.Vx * cos(car_state.psi) - car_state.Vy * sin(car_state.psi));
    double ky4 =
        (car_state.Vx * sin(car_state.psi) + car_state.Vy * cos(car_state.psi));
    double kpsi4 = car_state.Om;
    double kVx4 = car_state.accel;
    // Runge modifications:

    kx1 = (kx1 + 2 * kx2 + 2 * kx3 + kx4) / 6.0;
    ky1 = (ky1 + 2 * ky2 + 2 * ky3 + ky4) / 6.0;
    kpsi1 = (kpsi1 + 2 * kpsi2 + 2 * kpsi3 + kpsi4) / 6.0;
    kVx1 = (kVx1 + 2 * kVx2 + 2 * kVx3 + kVx4) / 6.0;
  }

  car_state.x = x0 + kx1 * dt;
  car_state.y = y0 + ky1 * dt;
  car_state.psi = psi0 + kpsi1 * dt;
  car_state.Vx = Vx0 + kVx1 * dt;
  car_state.Om = car_state.Vx * tan(car_state.steer_alpha) / car_par.L;
  car_state.Vy = car_state.Om * car_par.b;

  car_state.dVxdt = kVx1;

  if (((car_state.Vx < 0.0) && (fl_reverse == 0)) ||
      ((car_state.Vx > 0.0) && (fl_reverse > 0))) {
    car_state.x = x0;
    car_state.y = y0;
    car_state.psi = psi0;
    car_state.Vx = 0.0;
    car_state.Vy = 0.0;
    car_state.Om = 0.0;

    car_state.dVxdt = 0.0;
  }

  nav_to_cog(1);
}
//-------------------------------------------------------------------------------------------------------------------
//-///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//-////////////////////////////////////
void DynamicCar::nav_to_cog(int fl_reverse) {
  double cs = cos(car_state.psi);
  double sn = sin(car_state.psi);

  if (fl_reverse == 0) {
    car_state.x = car_state.xc + car_par.b * cs;
    car_state.y = car_state.yc + car_par.b * sn;

  } else {
    car_state.xc = car_state.x - car_par.b * cs;
    car_state.yc = car_state.y - car_par.b * sn;
  }
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//-/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//---/////////////////////////////////////////////////
void DynamicCar::init_nav(double x, double y, double psi, double v) {
  car_state.x = x;
  car_state.y = y;
  car_state.psi = psi;
  car_state.Vx = v;

  nav_to_cog(1);

  //---------------

  car_state.OmWf = car_state.Vx / car_par.frontWheelRad;
  car_state.OmWr = car_state.Vx / car_par.rearWheelRad;

  if (fabs(car_state.Vx) > 0.00001) car_state.state = 1;

  car_state.initialized += 1;

  //  std::cout << "V0 = " << car_state.Vx << std::endl;
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

static DynamicCar dcar(Eigen::Vector3d::Zero(3));

extern "C" void initModel(double x, double y, double psi, double v) {
  DynamicPar dynpar = DynamicPar();
  dcar.culc_par = dynpar.culc_par;
  dcar.car_par = dynpar.car_par;
  dcar.init_nav(x, y, psi, v);
  dcar.car_par.Om_nullTorque = 201.0;
  std::cout << "initModel(): " << dcar.car_par.Om_nullTorque << std::endl;
}

extern "C" void stepFunc(double steer, double accel_des, double dt,
                         int reverse_fl, double speed_des,
                         car_state_t* __dcar) {
  double ax = accel_des / dcar.car_par.mass;

  if (!dcar.car_state.initialized) {
    initModel(0, 0, 0, 0.0001);
  }

  dcar.dynamicNav_ev(steer, ax, dt, reverse_fl, 0.0);
  *__dcar = dcar.car_state;
  std::cout << " X: " << dcar.car_state.x << std::endl;
  //  __dcar->lat_slip = (__dcar->slipAng_r + __dcar->slipAng_f) / 2.0;
}
