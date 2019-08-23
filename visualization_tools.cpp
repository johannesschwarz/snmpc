//
// Created by johannes on 22/08/2019.
//

#include "visualization_tools.h"



namespace visualization_tools
{
    using namespace casadi;

    DM flipDM(const DM &matrix, int axis)
    {
        switch(axis)
        {
            case 1: return DM::vertcat({matrix(Slice(-1, 0, -1), Slice()), matrix(0, Slice())});
            case 2: return DM::horzcat({matrix(Slice(), Slice(-1, 0, -1)), matrix(Slice(), 0)});
        }
    }

    DMVector flipDMVector(DMVector matrix, int axis)
    {
        switch(axis)
        {
            case 1:
            case 2:
                for(int i = 0; i < matrix.size(); i++)
                {
                    matrix[i] = flipDM(matrix[i], axis);
                }
                return matrix;
            case 3:
                return reverse(matrix);

        }
    }

    DM extractAxis3(const DMVector &matrix, int i_axis_1, int i_axis_2)
    {
        int n_axis_3 = matrix.size();
        DM out = DM::zeros(n_axis_3, 1);
        for(int i_axis_3 = 0; i_axis_3 < n_axis_3; i_axis_3++)
        {
            out(i_axis_3) = matrix[i_axis_3](i_axis_1, i_axis_2);
        }
        return out;
    }

    DM interpolateLinear(const DM &x_data, const DM &y_data, const DM &x_sample)
    {
        DM y_sample = DM::zeros(x_sample.size());
        int n_sample = std::max(x_sample.size1(), x_sample.size2());
        int n_data = std::max(x_data.size1(), x_data.size2());

        double x_left;
        double x_right;
        double y_left;
        double y_right;
        double x_sample_cur;

        for(int i_sample = 0; i_sample < n_sample; i_sample++)
        {
            for(int i_data = 1; i_data < n_data; i_data++)
            {
                x_sample_cur = x_sample.nonzeros()[i_sample];
                x_left = x_data.nonzeros()[i_data - 1];
                x_right = x_data.nonzeros()[i_data];
                if(x_sample_cur >= x_left and x_sample_cur <= x_right)
                {
                    y_left = y_data.nonzeros()[i_data - 1];
                    y_right = y_data.nonzeros()[i_data];
                    y_sample(i_sample) = y_left + (y_right - y_left) / (x_right - x_left) * (x_sample_cur - x_left);
                }
            }
        }

        return y_sample;
    }

    DM integrateDiscretePath(const DM &x, const DM &y)
    {
        DM s_path = DM::zeros(x.size());
        int n_grid = std::max(x.size1(), x.size2());
        DM dxy_cur;

        // simply use Pythagoras
        for(int i = 1; i < n_grid; i++)
        {
            dxy_cur = DM::vertcat({x(i) - x(i-1), y(i) - y(i-1)});
            s_path(i) = s_path(i-1) + DM::norm_2(dxy_cur);
        }
        return s_path;
    }




//
//    std::ofstream traj_file;
//    traj_file.open("../log/sol_traj.txt", std::ios::trunc);
//    traj_file << "v_x, v_y, omega, x, y, theta, omega_w_f, omega_w_r, theta_aug, theta_dt_aug, phi, Fx_f, Fx_r, v_aug, info" << "\n";
//    for (int i_colloc = n_colloc - 1; i_colloc >= 0; i_colloc--)
//{
//    for (int i_state = 0; i_state < dimx_aug; i_state++)
//{
//    traj_file << opt_traj(i_state, i_colloc) << ",";
//}
//for (int i_ctrl = 0; i_ctrl < dimu_aug; i_ctrl++)
//{
//traj_file << opt_ctrl(i_ctrl, i_colloc) << ",";
//}
//if(i_colloc == n_colloc - 1){
//traj_file << n_colloc;
//} else if(i_colloc == n_colloc - 2){
//traj_file << dimx_aug;
//} else if(i_colloc == n_colloc - 3) {
//traj_file << dimu_aug;
//} else {
//traj_file << "0";
//}
//
//traj_file << "\n";
//}
//traj_file.close();

}