//
// Created by johannes on 22/08/2019.
//

#include "casadi/casadi.hpp"


#ifndef JOHANNES_MPC_VISUALIZATION_TOOLS_H
#define JOHANNES_MPC_VISUALIZATION_TOOLS_H

using namespace casadi;

namespace visualization_tools
{

    DM flipDM(const DM &matrix, int axis);

    DMVector flipDMVector(DMVector matrix, int axis);

    DM extractAxis3(const DMVector &matrix, int i_axis_1, int i_axis_2);

    DM interpolateLinear(const DM &x_data, const DM &y_data, const DM &x_sample);

    template <typename Path>
    DM evaluatePath(Path path, const DM &grid)
    {

        int n_grid = std::max(grid.size1(), grid.size2()); //DM::size1(grid);
        DM xy_path = DM::zeros(2, n_grid);
        // create Path function
        SX t_path = SX::sym("t_path");
        SX path_sym = path(SXVector{t_path})[0];
        Function path_fun = Function("path_fun",{t_path}, {path_sym});
        // evaluate
        for(int i = 0; i < n_grid; i++)
        {
            xy_path(Slice(), i) = path_fun(grid(i))[0];
        }
        // return
        return xy_path;
    }

    template <typename Path>
    DM integrateFunPath(Path path, const DM &grid)
    {
        // create Path function
        SX t_path = SX::sym("t_path");
        SX path_sym = path(SXVector{t_path})[0];
        Function path_fun = Function("path_fun",{t_path}, {path_sym});

        // some variables
        DM s_path = DM::zeros(grid.size());
        int n_grid = std::max(grid.size1(), grid.size2());
        DM dxy_cur;

        // simply use Pythagoras
        for(int i = 1; i < n_grid; i++)
        {
            dxy_cur = path_fun(grid(i))[0] - path_fun(grid(i-1))[0];
            s_path(i) = s_path(i-1) + DM::norm_2(dxy_cur);
        }

        return s_path;
    }


}

#endif //JOHANNES_MPC_VISUALIZATION_TOOLS_H
