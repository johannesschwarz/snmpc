#include "kinematic_bicycle.hpp"
#include "nmpf.hpp"
#include "matplotlibcpp.h"
#include "integrator.h"
#include "visualization_tools.h"

namespace plt = matplotlibcpp;
using namespace casadi;
namespace vis = visualization_tools;

struct Path
{
    SXVector operator()(const SXVector &arg)
    {
        // simple circle, tangent pointing in y direction at phi=0 starting at [0,0]
        SX phi = SX::sym("phi");
        double radius = 50; // [m]
        SX x = radius * (cos(phi) - 1);
        SX y = radius * sin(phi);
        SX Path = SX::vertcat({x,y});
        Function path = Function("path", {phi},{Path});
        return path(arg);
    }
};

int main(int argc, char **argv)
{
    /** variables **/
    // memory
    DM state;               // current state
    DM augmented_state;     // current augmented state
    DM controls;            // current control
    DM opt_traj;            // current optimal trajectory
    DM opt_ctrl;            // current optimal control
    DMVector opt_traj_hist; // all past trajectories
    DMVector opt_ctrl_hist; // all past controls
    // model dimensions
    const int dimx = 3;         // state dimension without augmentation
    const int dimu = 2;         // input dimension without augmentation
    // simulator
    const int n_sim = 1000;                            // simulation steps
    const double dt_sim = 1.0 / 50.0;               // timestep for simulation
    Dict ode_params({{"tf", dt_sim}});              // parameters for ode solver
    KinematicBicycle kinematic_bicycle;                       // dynamics object
    Function ode_rhs = kinematic_bicycle.getDynamics();  // get the RHS of the robot ODE
    ODESolver ode_solver(ode_rhs, ode_params);      // create solver object
    // controller
    const double tf = 4.0;      // [s] time horizon for MPC
    const int n_segments = 5;
    const int poly_order = 4;
    polympc::nmpf<KinematicBicycle, Path, dimx, dimu, n_segments, poly_order> controller(tf);  // nmpf object


    /** state and control constraints, initial state and reference velocity*/
    // controller constraints
    const double phi_max = M_PI / 6.0;
    const double v_min = 1.0e-1; // [m/s]
    const double v_max = 50.0;
    // controls: [v, phi, v_aug]
    DM lbu = DM::vertcat({v_min, -phi_max, -DM::inf()});
    DM ubu = DM::vertcat({v_max, phi_max, DM::inf()});
    controller.setLBU(lbu);
    controller.setUBU(ubu);
    // state constraints
    const double xy_max = 1.0e3;
    // states: [x, y, theta, theta_aug, theta_dt_aug]
    DM lbx = DM::vertcat({-xy_max, -xy_max, -DM::inf(), -DM::inf(), -DM::inf()});
    DM ubx = DM::vertcat({xy_max, xy_max, DM::inf(), DM::inf(), DM::inf()});
    controller.setLBX(lbx);
    controller.setLBU(lbu);
    // reference velocity
    const double phi_dt_ref = 10.0 / 50.0;
    controller.setReferenceVelocity(phi_dt_ref);
    // set initial state
    state = DM::vertcat({0, 0, M_PI_2});
    augmented_state = DM::vertcat({state, 0, phi_dt_ref});

    /*
     * TODO: What is missing?
     * o Set Cost function from external
     * o make initial wheel speed dependent on actual wheel radius
     * o having other paths, this includes calculating the arc length numerically
     */

    for(int i_sim = 0; i_sim < n_sim; i_sim++)
    {
        /** Compute Control **/
        // augment the state
        // compute control and get the trajectories
        controller.computeControl(augmented_state);
        opt_ctrl  = vis::flipDM(controller.getOptimalControl(), 2);
        opt_traj = vis::flipDM(controller.getOptimalTrajetory(), 2);


        // extract first control
        controls = opt_ctrl(Slice(0,dimu), 0);

        /** Simulate **/
        state = ode_solver.solve(state, controls, dt_sim);

        /** Prepare next loop **/
        // augment state
        augmented_state = DM::vertcat({state, opt_traj(Slice(opt_traj.size1() - 2, opt_traj.size1()), 0)});

        /** save data **/
        opt_ctrl_hist.push_back(opt_ctrl);
        opt_traj_hist.push_back(opt_traj);

    }

    /** the plotting part **/
    // prepare path function
    Path path;

    // evaluate desired path
    const int n_eval = 500;
    DM theta_path_des = DM::linspace(-M_PI_4, M_PI, n_eval);
    DM xy_path_des = vis::evaluatePath<Path>(path, theta_path_des);
    DM x_path_des = xy_path_des(0, Slice());
    DM y_path_des = xy_path_des(1, Slice());
    DM s_path_des = vis::integrateFunPath<Path>(path, theta_path_des);
    // extract driven states
    DM x_real = vis::extractAxis3(opt_traj_hist, 0, 0);
    DM y_real = vis::extractAxis3(opt_traj_hist, 1, 0);
    DM theta_head_real = vis::extractAxis3(opt_traj_hist, 2, 0);
    DM theta_aug_real = vis::extractAxis3(opt_traj_hist, -2, 0);
    DM theta_aug_dt_real = vis::extractAxis3(opt_traj_hist, -1, 0);
    // extract applied controls
    DM phi_real = vis::extractAxis3(opt_ctrl_hist, 1, 0);
    DM v_real = vis::extractAxis3(opt_ctrl_hist, 0, 0);
    DM v_aug_real = vis::extractAxis3(opt_ctrl_hist, 2, 0);
    // interpolate path arc length at collocation points
    DM s_path_colloc_real = vis::interpolateLinear(theta_path_des, s_path_des, theta_aug_real);


    // extract data from the first optimization
    // states at first
    DM x_pred = opt_traj_hist.front()(0, Slice());
    DM y_pred = opt_traj_hist.front()(1, Slice());
    DM theta_aug_pred = opt_traj_hist.front()(-2, Slice());
    DM theta_aug_dt_pred = opt_traj_hist.front()(-1, Slice());
    DM xy_path_colloc_pred = vis::evaluatePath<Path>(path, theta_aug_pred);
    DM x_colloc_pred = xy_path_colloc_pred(0, Slice());
    DM y_colloc_pred = xy_path_colloc_pred(1, Slice());
    // controls second
    DM phi_pred = opt_ctrl_hist.front()(1, Slice());
    DM v_pred= opt_ctrl_hist.front()(0, Slice());
    DM v_aug_pred = opt_ctrl_hist.front()(2, Slice());
    // interpolate patch arc length at collocation points
    DM s_path_colloc_pred = vis::interpolateLinear(theta_path_des, s_path_des, theta_aug_pred);



    // plot xy trajectory
    plt::figure();
    plt::named_plot("optimized trajectory", x_pred.nonzeros(), y_pred.nonzeros());
    plt::named_plot("desired path", x_path_des.nonzeros(), y_path_des.nonzeros());
    plt::named_plot("desired path position at collocation points", x_colloc_pred.nonzeros(), y_colloc_pred.nonzeros(), ".");
    plt::named_plot("driven path", x_real.nonzeros(), y_real.nonzeros());
    plt::axis("equal");
    plt::xlabel("x");
    plt::ylabel("y");
    plt::title("xy-coordinates");
    plt::legend();

    plt::figure();
    plt::named_plot("desired path", x_path_des.nonzeros(), y_path_des.nonzeros());
    plt::named_plot("driven path", x_real.nonzeros(), y_real.nonzeros());
    int n_stride = 10;
    for(int i = 0; i < opt_traj_hist.size(); i = i + n_stride)
    {
        DM x_pred_cur = opt_traj_hist[i](0, Slice());
        DM y_pred_cur = opt_traj_hist[i](1, Slice());
        plt::plot(x_pred_cur.nonzeros(), y_pred_cur.nonzeros(), "c");
    }
    plt::axis("equal");
    plt::xlabel("x");
    plt::ylabel("y");
    plt::title("xy-coordinates");
    plt::legend();

    // plot steering angle for entire trajectory
    plt::figure();
    plt::plot(s_path_colloc_real.nonzeros(), phi_real.nonzeros());
    plt::xlabel("arc length");
    plt::ylabel("steering angle [rad]");
    plt::title("steering angle");
    // plot v input
    plt::figure();
    plt::plot(s_path_colloc_real.nonzeros(), v_real.nonzeros());
    plt::xlabel("arc length");
    plt::ylabel("velocity [m/s]");
    plt::title("velocity input");

    // plot predicted quantities
    plt::figure();
    plt::plot(s_path_colloc_pred.nonzeros(), phi_pred.nonzeros());
    plt::xlabel("arc length");
    plt::ylabel("steering angle [rad]");
    plt::title("phi predicted");

    plt::figure();
    plt::plot(s_path_colloc_pred.nonzeros(), v_pred.nonzeros());
    plt::xlabel("arc length");
    plt::ylabel("velocity [m/s]");
    plt::title("velocity input prediction");

    plt::figure();
    plt::named_plot("predicted", s_path_colloc_pred.nonzeros(), theta_aug_dt_pred.nonzeros());
    plt::named_plot("desired", s_path_colloc_pred.nonzeros(), (DM::ones(theta_aug_dt_pred.size()) * phi_dt_ref).nonzeros());
    plt::xlabel("arc length");
    plt::ylabel("augmented velocity");
    plt::title("theta_dt predicted");
    plt::legend();

    plt::figure();
    plt::plot(s_path_colloc_pred.nonzeros(), v_aug_pred.nonzeros());
    plt::xlabel("arc length");
    plt::ylabel("augmented input");
    plt::title("v_aug predicted");

    plt::show();



    return 0;

}


