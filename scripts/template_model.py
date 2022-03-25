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


def template_model(multi_agent_params):
    """
    --------------------------------------------------------------------------
    template_model: Variables / RHS / AUX
    --------------------------------------------------------------------------
    
    """
    n_x = multi_agent_params['n_x']
    n_o = multi_agent_params['n_o']

    g = 9.81
    delT = 0.1
    
    A = np.array([[0,0,0,1,0,0,0,0,0,0,0,0],
                  [0,0,0,0,1,0,0,0,0,0,0,0],
                  [0,0,0,0,0,1,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,g,0,0,0,0],
                  [0,0,0,0,0,0,-g,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,1,0,0],
                  [0,0,0,0,0,0,0,0,0,0,1,0],
                  [0,0,0,0,0,0,0,0,0,0,0,1],
                  [0,0,0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0,0,0],
                  [0,0,0,0,0,0,0,0,0,0,0,0],])
    
    B = np.array([[0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [1,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,0,0,0],
                  [0,1,0,0],
                  [0,0,1,0],
                  [0,0,0,1],])
    



    model_type = 'discrete' # either 'discrete' or 'continuous'
    model = do_mpc.model.Model(model_type)

    # Simple oscillating masses example with two masses and two inputs.
    # States are the position and velocitiy of the two masses.

    # States struct (optimization variables):
    for i in range(n_x):
        x_var_name = 'x' + str(i+1)
        u_var_name = 'u' + str(i+1)
        G_var_name = 'G' + str(i+1)
        H_var_name = 'H' + str(i+1)
        _x = model.set_variable(var_type='_x', var_name=x_var_name, shape=(12,1))
        _u = model.set_variable(var_type='_u', var_name=u_var_name, shape=(4,1))
        x_next = _x + delT*(A@_x+B@_u)
        model.set_rhs(x_var_name, x_next)
        _G = model.set_variable(var_type='_tvp', var_name=G_var_name, shape=(10,3))
        _H = model.set_variable(var_type='_tvp', var_name=H_var_name, shape=(10,1))

        for j in (j for j in range(n_x) if j!=i):
            lambda_var_name = 'lambda' + str(i+1) + '_for_agent' + str(j+1)
            _lambda = model.set_variable(var_type='_u', var_name=lambda_var_name, shape=(10,1))

        if n_o!=0:
            for k in range(n_o):
                lambda_var_name = 'lambda' + str(i+1) + '_for_obst' + str(k+1)
                _lambda = model.set_variable(var_type='_u', var_name=lambda_var_name, shape=(6,1))
    for k in range(n_o):
        G_var_name = 'obsG' + str(k+1)
        H_var_name = 'obsH' + str(k+1)
        _G = model.set_variable(var_type='_tvp', var_name=G_var_name, shape=(6,3))
        _H = model.set_variable(var_type='_tvp', var_name=H_var_name, shape=(6,1))
 

    # Input struct (optimization variables):
    # _u = model.set_variable(var_type='_u', var_name='u', shape=(4,1))
    # _lambda = model.set_variable(var_type='_u', var_name='lambda', shape=(6,1))
    
    # _lambda1 = model.set_variable(var_type='_u', var_name='lambda1', shape=(6,1))
    # _lambda2 = model.set_variable(var_type='_u', var_name='lambda2', shape=(6,1))
    # _lambda3 = model.set_variable(var_type='_u', var_name='lambda3', shape=(6,1))
    # _lambda4 = model.set_variable(var_type='_u', var_name='lambda4', shape=(6,1))
    # _lambda5 = model.set_variable(var_type='_u', var_name='lambda5', shape=(6,1))

    # Set expression. These can be used in the cost function, as non-linear constraints
    # or just to monitor another output.
    # model.set_expression(expr_name='cost', expr=sum1(_x**2))
    

    model.setup()

    return model
