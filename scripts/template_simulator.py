#
#   This file is part of do-mpc
#
#   do-mpc: An environment for the easy, modular and efficient implementation of
#        robust nonlinear model predictive control
#
#   Copyright (c) 2014-2019 Sergio Lucia, Alexandru Tatulea-Codrean
#                        TU Dortmund. All rights reserved
#
#   do-mpc is free software: you can redistribute it and/or modify
#   it under the terms of the GNU Lesser General Public License as
#   published by the Free Software Foundation, either version 3
#   of the License, or (at your option) any later version.
#
#   do-mpc is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU Lesser General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with do-mpc.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc


def template_simulator(model,mpc,multi_agent_params):
    """
    --------------------------------------------------------------------------
    template_optimizer: tuning parameters
    --------------------------------------------------------------------------
    """
    simulator = do_mpc.simulator.Simulator(model)
    data = do_mpc.data.MPCData(model)

    n_x = multi_agent_params['n_x']
    n_o = multi_agent_params['n_o']
    n_horizon = multi_agent_params['n_horizon']
    goal = multi_agent_params['goal_pos']
    AllObsG = multi_agent_params['obst_G']
    AllObsH = multi_agent_params['obst_H']


    simulator.set_param(t_step = 0.1)

    tvp_temp = simulator.get_tvp_template()
    def get_G_and_H(model):
        for i in range(n_x):
            x_var_name = 'x' + str(i+1)
            u_var_name = 'u' + str(i+1)
            # _x = model.x[x_var_name]
            # _u = model.x[u_var_name]
            G_var_name = 'G' + str(i+1)
            H_var_name = 'H' + str(i+1)
            # G = np.zeros((6,3))
            # G[0,0] = 1
            # G[1,0] = -1
            # G[2,1] = 1
            # G[3,1] = -1
            # G[4,2] = 1
            # G[5,2] = -1


            G = np.zeros((10,3))
            G[0,0] = 1
            G[0,1] = 1
            G[1,0] = -1
            G[1,1] = 1
            G[2,0] = -1
            G[2,1] = -1
            G[3,0] = 1
            G[3,1] = -1
            G[4,0] = 1
            G[5,0] = -1
            G[6,1] = 1
            G[7,1] = -1
            G[8,2] = 1
            G[9,2] = -1
            tvp_temp[G_var_name]=G
            try: 
                X = data.prediction(('_x',x_var_name,0),0)
                Y = data.prediction(('_x',x_var_name,1),0)
                Z = data.prediction(('_x',x_var_name,2),0)
            except: 
                pos = multi_agent_params['init_pos']
                X = pos[0,i]
                Y = pos[1,i]
                Z = pos[2,i]
                # X = model.x[x_var_name,0]
                # Y = model.x[x_var_name,1]
                # Z = model.x[x_var_name,2]
            # H = np.ones((6,1))
            # H[0,0] = X+1
            # H[1,0] = -(X-1)
            # H[2,0] = Y+1
            # H[3,0] = -(Y-1)
            # H[4,0] = Z+4
            # H[5,0] = -(Z-4)

            H = np.ones((10,1))
            H[0,0] = Y+X+1
            H[1,0] = -X+Y+1
            H[2,0] = -X-Y+1
            H[3,0] = X-Y+1
            H[4,0] = X+1
            H[5,0] = -(X-1)
            H[6,0] = Y+1
            H[7,0] = -(Y-1)
            H[8,0] = Z+1
            H[9,0] = -(Z-1)


            tvp_temp[H_var_name]=H
        return tvp_temp
            
    simulator.set_tvp_fun(get_G_and_H)

    simulator.setup()

    return simulator
