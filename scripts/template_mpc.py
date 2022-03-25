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


def template_mpc(model,multi_agent_params):
    """
    --------------------------------------------------------------------------
    template_mpc: tuning parameters
    --------------------------------------------------------------------------
    """
    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_robust': 0,
        'n_horizon': multi_agent_params['n_horizon'],
        't_step': 0.1,
        'state_discretization': 'discrete'
    }
    mpc.set_param(**setup_mpc)
    mpc.set_param(nlpsol_opts = {'ipopt.linear_solver': 'MA27', 'ipopt.max_iter': 50})
    mpc.set_param(store_full_solution=True)

    surpress_ipopt = {'ipopt.print_level':0, 'ipopt.sb': 'yes', 'print_time':0}
    #mpc.set_param(nlpsol_opts = surpress_ipopt)

    data = do_mpc.data.MPCData(model)

    n_x = multi_agent_params['n_x']
    n_o = multi_agent_params['n_o']
    n_horizon = setup_mpc['n_horizon']
    goal = multi_agent_params['goal_pos']
    AllObsG = multi_agent_params['obst_G']
    AllObsH = multi_agent_params['obst_H']



    
    
    # X = model.x['x', 0]
    # Y = model.x['x', 1]
    # Z = model.x['x', 2]
    # X_dot = model.x['x', 3]
    # Y_dot = model.x['x', 4]
    # Z_dot = model.x['x', 5]
    # Phi = model.x['x', 6]
    # Theta = model.x['x', 7]
    # Psi = model.x['x', 8]
    # Phi_dot = model.x['x', 9]
    # Theta_dot = model.x['x',10]
    # Psi_dot = model.x['x', 11]
    
    # _x = model.x['x']
    # _u = model.u['u']
 
    #_lambda = model.u['lambda']
    
    # '''Defining the Obstacle'''
    # G = np.zeros((6,3))
    # G[0,0] = 1
    # G[1,0] = -1
    # G[2,1] = 1
    # G[3,1] = -1
    # G[4,2] = 1
    # G[5,2] = -1
    
    # H = np.ones((6,1))
    # H[0,0] = 7
    # H[1,0] = -3
    # H[2,0] = 7
    # H[3,0] = -3
    # H[4,0] = 7
    # H[5,0] = -3
    
   
    # H1 = np.ones((6,1))
    # H1[0,0] = 3
    # H1[1,0] = 0
    # H1[2,0] = 8
    # H1[3,0] = -2
    # H1[4,0] = 7
    # H1[5,0] = -3
    
    # H2 = np.ones((6,1))
    # H2[0,0] = 10
    # H2[1,0] = 0
    # H2[2,0] = 8
    # H2[3,0] = -2
    # H2[4,0] = 3
    # H2[5,0] = 0
    
    # H3 = np.ones((6,1))
    # H3[0,0] = 10
    # H3[1,0] = -7
    # H3[2,0] = 8
    # H3[3,0] = -2
    # H3[4,0] = 7
    # H3[5,0] = -3
    
    # H4 = np.ones((6,1))
    # H4[0,0] = 10
    # H4[1,0] = 0
    # H4[2,0] = 8
    # H4[3,0] = -2
    # H4[4,0] = 10
    # H4[5,0] = -7
    
    # H5 = np.ones((6,1))
    # H5[0,0] = 10
    # H5[1,0] = -7
    # H5[2,0] = 2
    # H5[3,0] = 0
    # H5[4,0] = 10
    # H5[5,0] = 0
    
    
    
    
    # Q = np.zeros((12,12))
    # Q[0,0] = 1
    # Q[1,1] = 1
    # Q[2,2] = 1
    # Q[3,3] = 1
    # Q[4,4] = 1
    # Q[5,5] = 1
    # Q[6,6] = 5
    # Q[7,7] = 5
    # Q[8,8] = 5
    # Q[9,9] = 1
    # Q[10,10] = 1
    # Q[11,11] = 1
    
    # R = np.zeros((4,4))
    # R[0,0] = 0.1
    # R[1,1] = 5
    # R[2,2] = 5
    # R[3,3] = 5
    
    
    Q = np.zeros((12,12))
    Q[0,0] = 3
    Q[1,1] = 3
    Q[2,2] = 3
    Q[3,3] = 1.4
    Q[4,4] = 1.4
    Q[5,5] = 1
    Q[6,6] = 0.1
    Q[7,7] = 0.1
    Q[8,8] = 0.1
    Q[9,9] = 1
    Q[10,10] = 1
    Q[11,11] = 0.1

    Q[0,3] = 0.00
    Q[1,4] = 0.00
    Q[2,5] = 0.00
    
    R = np.zeros((4,4))
    R[0,0] = 5
    R[1,1] = 6.5
    R[2,2] = 6.5
    R[3,3] = 6.5
    
    
    
    
    # x_ref = np.zeros((12,1))
    # x_ref[2] = 10
    # x_ref[1] = 0
    # x_ref[0] = 0
    # err_x = _x - x_ref
    
    # mterm = (err_x.T)@Q@err_x #+ (_u.T)@R@_u
    # lterm = mterm + (_u.T)@R@_u
    # #(_u.T)@R@_u
    
    # mterm = model.aux['cost']
    # lterm = model.aux['cost'] # terminal cost

    # mpc.set_objective(mterm=mterm, lterm=lterm)
    # mpc.set_rterm(u=1*np.ones((4,1)))

    max_x = np.array([[30.0], [30.0], [30.0], [1.5], [1.5], [0.5], [0.3], [0.3], [0.3], [0.5], [0.5], [0.5]])
    max_u = np.array([[2],[1],[1],[1]])
    #min_u = np.array([[-0.2],[-2],[2],[2]])

    # mpc.bounds['lower','_x','x'] = -max_x
    # mpc.bounds['upper','_x','x'] =  max_x
    
    # # mpc.bounds['lower','_u','lambda1'] = 0.05*np.ones((6,1))
    # # mpc.bounds['lower','_u','lambda2'] = 0.05*np.ones((6,1))
    # # mpc.bounds['lower','_u','lambda3'] = 0.05*np.ones((6,1))
    # # mpc.bounds['lower','_u','lambda4'] = 0.05*np.ones((6,1))
    # # mpc.bounds['lower','_u','lambda5'] = 0.05*np.ones((6,1))
    
    # # mpc.bounds['lower','_u','lambda'] = 0.1*np.ones((6,1))
    # mpc.bounds['lower','_u','u'] = -max_u
    # mpc.bounds['upper','_u','u'] =  max_u
    

    # _z = np.array([[_x[0]],
    #                 [_x[1]],
    #                 [_x[2]]])
    

    # _obst_con = -((G@_z - H).T)@_lambda
    # GL = (G.T)@_lambda
    # _obst_con_abs = norm_2(GL) 
    # print(GL)
    # print(_lambda)
    # print(_obst_con_abs)
    
    # x_simple_obs = np.array([[0],
    #                          [0],
    #                          [5]])
    
    # simple_obs = _z - x_simple_obs
    # simple_obs_cons = norm_2(simple_obs)
    # #simple_obs_cons = simple_obs[0] + simple_obs[1] + simple_obs[2]
    # #simple_obs_cons = (simple_obs[0])**2 + (simple_obs[1])**2 + (simple_obs[2])**2 

    # #mpc.set_nl_cons('simple_obs_cons', -simple_obs_cons, ub=-2)
    
    
    #mpc.set_nl_cons('obst_con', _obst_con, ub=0,  soft_constraint=True, penalty_term_cons=100)
    #mpc.set_nl_cons('obst_con_abs', _obst_con_abs, ub=1, soft_constraint=True, penalty_term_cons=100)


    tvp_temp = mpc.get_tvp_template()
    def get_G_and_H(model):
        for i in range(n_x):
            x_var_name = 'x' + str(i+1)
            u_var_name = 'u' + str(i+1)
            # _x = model.x[x_var_name]
            # _u = model.x[u_var_name]
            G_var_name = 'G' + str(i+1)
            H_var_name = 'H' + str(i+1)
            G = np.zeros((10,3))
            # G[0,0] = 1
            # G[1,0] = -1
            # G[2,1] = 1
            # G[3,1] = -1
            # G[4,2] = 1
            # G[5,2] = -1


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
            print(G)
           
            # _X = mpc.data.prediction(('_x',x_var_name,0))
            # _Y = mpc.data.prediction(('_x',x_var_name,1))
            # _Z = mpc.data.prediction(('_x',x_var_name,2))
            for k in range(n_horizon+1):
                tvp_temp['_tvp',k,G_var_name]=G

                try: 
                    _X = mpc.data.prediction(('_x',x_var_name,0))
                    _Y = mpc.data.prediction(('_x',x_var_name,1))
                    _Z = mpc.data.prediction(('_x',x_var_name,2))
                    X = _X[0,k,0]
                    Y = _Y[0,k,0]
                    Z = _Z[0,k,0]
                    if X==0 and Y==0 and Z==0:
                        pos = multi_agent_params['init_pos']
                        X = pos[0,i]
                        Y = pos[1,i]
                        Z = pos[2,i]
                        #print('inside if')
                    
                    # print(k)
                except: 
                    pos = multi_agent_params['init_pos']
                    X = pos[0,i]
                    Y = pos[1,i]
                    Z = pos[2,i]
                    #print('exception!')
                    # print(X)

                #print(X)
                # try: 
                #     _X = mpc.data.prediction(('_x',x_var_name,0))
                #     _Y = mpc.data.prediction(('_x',x_var_name,1))
                #     _Z = mpc.data.prediction(('_x',x_var_name,2))
                #     X = _X[0,k,0]
                #     Y = _Y[0,k,0]
                #     Z = _Z[0,k,0]
                #     print(X)
                    
                #     # print(k)
                # except: 
                #     pos = multi_agent_params['init_pos']
                #     X = pos[0,i]
                #     Y = pos[1,i]
                #     Z = pos[2,i]
                #     print('exception!')
                #     print(X)
                # X = model.x[x_var_name,0]
                # Y = model.x[x_var_name,1]
                # Z = model.x[x_var_name,2]
                # H = np.ones((6,1))
                # H[0,0] = X+1
                # H[1,0] = -(X-1)
                # H[2,0] = Y+1
                # H[3,0] = -(Y-1)
                # H[4,0] = Z+1
                # H[5,0] = -(Z-1)

                H = np.ones((10,1))
                a=0.3
                H[0,0] = Y+X+a
                H[1,0] = -X+Y+a
                H[2,0] = -X-Y+a
                H[3,0] = X-Y+a
                H[4,0] = X+a
                H[5,0] = -(X-a)
                H[6,0] = Y+a
                H[7,0] = -(Y-a)
                H[8,0] = Z+0.5
                H[9,0] = -(Z-0.5)


                tvp_temp['_tvp',k,H_var_name]=H

                # print(G)
                # print(H)                
        if n_o!=0:
            for i in range(n_o):
                print('inside n_o')
                G_var_name = 'obsG' + str(i+1)
                H_var_name = 'obsH' + str(i+1)
                G = np.array(AllObsG[i])
                H = np.array(AllObsH[i])
                print(G)
                for k in range(n_horizon+1):
                    tvp_temp['_tvp',k,G_var_name]=G
                    print('inside k')
                    tvp_temp['_tvp',k,H_var_name]=H
        return tvp_temp

            
    mpc.set_tvp_fun(get_G_and_H)




    # for i in range(n_x):
    #     x_var_name = 'x' + str(i+1)
    #     u_var_name = 'u' + str(i+1)
    #     # _x = model.x[x_var_name]
    #     # _u = model.x[u_var_name]
    #     G_var_name = 'G' + str(i+1)
    #     H_var_name = 'H' + str(i+1)
    #     G = np.zeros((6,3))
    #     G[0,0] = 1
    #     G[1,0] = -1
    #     G[2,1] = 1
    #     G[3,1] = -1
    #     G[4,2] = 1
    #     G[5,2] = -1
    #     for k in range(n_horizon+1):
    #         tvp_temp['_tvp',k,G_var_name]=G
    #         # X = data.prediction(('_x',x_var_name,0),k)
    #         # Y = data.prediction(('_x',x_var_name,1),k)
    #         # Z = data.prediction(('_x',x_var_name,2),k)
    #         X = model.x[x_var_name,0]
    #         Y = model.x[x_var_name,1]
    #         Z = model.x[x_var_name,2]
    #         H = np.ones((6,1))
    #         H[0,0] = X+1
    #         H[1,0] = -(X-1)
    #         H[2,0] = Y+1
    #         H[3,0] = -(Y-1)
    #         H[4,0] = Z+4
    #         H[5,0] = -(Z-4)
    #         tvp_temp['_tvp',k,H_var_name]=H
    # return tvp_temp





















    lterm = 0
    mterm = 0

    for i in range(n_x):
        x_var_name = 'x' + str(i+1)
        u_var_name = 'u' + str(i+1)
        _x = model.x[x_var_name]
        _u = model.u[u_var_name]
        _z = np.array([[_x[0]],
                       [_x[1]],
                       [_x[2]]])
        x_ref = np.zeros((12,1))
        x_ref[0] = goal[0,i]
        x_ref[1] = goal[1,i]
        x_ref[2] = goal[2,i]
        err_x = _x - x_ref
        # if i == 10:
        #     mterm = mterm + (err_x.T)@(1*Q)@err_x #+ (err_x[0]*err_x[0] + err_x[1]*err_x[1] + err_x[2]*err_x[2])/sqrt(err_x[3]*err_x[3] + err_x[4]*err_x[4] + err_x[5]*err_x[5] + 0.01)
        #     lterm = lterm + (err_x.T)@(1*Q)@err_x + (_u.T)@R@_u #+ (err_x[0]*err_x[0] + err_x[1]*err_x[1] + err_x[2]*err_x[2])/sqrt(err_x[3]*err_x[3] + err_x[4]*err_x[4] + err_x[5]*err_x[5] + 0.01)
        # else: 
        #     mterm = mterm + (err_x.T)@(1*Q)@err_x #+ 0.1*(err_x[0]*err_x[0] + err_x[1]*err_x[1] + err_x[2]*err_x[2])/sqrt(err_x[3]*err_x[3] + err_x[4]*err_x[4] + err_x[5]*err_x[5] + 0.1)
        #     lterm = lterm + (err_x.T)@(1*Q)@err_x + (_u.T)@R@_u #+ 0.1*(err_x[0]*err_x[0] + err_x[1]*err_x[1] + err_x[2]*err_x[2])/sqrt(err_x[3]*err_x[3] + err_x[4]*err_x[4] + err_x[5]*err_x[5] + 0.1)
        
        mterm = mterm + (err_x.T)@(1*Q)@err_x #+ (err_x[0]*err_x[0] + err_x[1]*err_x[1] + err_x[2]*err_x[2])/sqrt(err_x[3]*err_x[3] + err_x[4]*err_x[4] + err_x[5]*err_x[5] + 0.01)
        lterm = lterm + (err_x.T)@(1*Q)@err_x + (_u.T)@R@_u #+ 0.01*(err_x[0]*err_x[0] + err_x[1]*err_x[1] + err_x[2]*err_x[2])/sqrt(err_x[3]*err_x[3] + err_x[4]*err_x[4] + err_x[5]*err_x[5] + 1)
        


        mpc.bounds['lower','_x',x_var_name] = -max_x
        mpc.bounds['upper','_x',x_var_name] =  max_x
        mpc.bounds['lower','_u',u_var_name] = -max_u
        mpc.bounds['upper','_u',u_var_name] =  max_u
        mpc.set_rterm(u1 = 5.5*np.ones((4,1)))
        mpc.set_rterm(u2 = 5.5*np.ones((4,1)))
        mpc.set_rterm(u3 = 5.5*np.ones((4,1)))
        mpc.set_rterm(u4 = 5.5*np.ones((4,1)))
        mpc.set_rterm(u5 = 5.5*np.ones((4,1)))
        mpc.set_rterm(u6 = 5.5*np.ones((4,1)))
    

        for j in (j for j in range(n_x) if j!=i):
            lambda_var_name = 'lambda' + str(i+1) + '_for_agent' + str(j+1)
            G_var_name = 'G' + str(j+1)
            H_var_name = 'H' + str(j+1)
            G = model.tvp[G_var_name]
            H = model.tvp[H_var_name]
            mpc.bounds['lower','_u',lambda_var_name] = 0.1*np.ones((10,1))
            _lambda = model.u[lambda_var_name]
            _obst_con = -((G@_z - H).T)@_lambda
            GL = (G.T)@_lambda
            _obst_con_abs = norm_2(GL)
            obst_con_var_name = 'obst_con_pt' + str(i+1) + '_other_agent' + str(j+1)
            obst_con_abs_var_name = 'obst_con_abs_pt' + str(i+1) + '_other_agent' + str(j+1)
            mpc.set_nl_cons(obst_con_var_name, _obst_con, ub=-0.5,  soft_constraint=True, penalty_term_cons=100)
            mpc.set_nl_cons(obst_con_abs_var_name, _obst_con_abs, ub=1, soft_constraint=True, penalty_term_cons=100)
            # print('constraint value: ')
            # print(_obst_con_abs)

        if n_o!=0:
            for k in range(n_o):
                lambda_var_name = 'lambda' + str(i+1) + '_for_obst' + str(k+1)
                G_var_name = 'obsG' + str(k+1)
                H_var_name = 'obsH' + str(k+1)
                G = model.tvp[G_var_name]
                H = model.tvp[H_var_name]
                # obstG = np.zeros((6,3))
                # obstG[0,0] = 1
                # obstG[1,0] = -1
                # obstG[2,1] = 1
                # obstG[3,1] = -1
                # obstG[4,2] = 1
                # obstG[5,2] = -1
                
                # obstH = np.ones((6,1))
                # obstH[0,0] = 7
                # obstH[1,0] = -3
                # obstH[2,0] = 7
                # obstH[3,0] = -3
                # obstH[4,0] = 7
                # obstH[5,0] = -3
                # G = obstG
                # H = obstH
                mpc.bounds['lower','_u',lambda_var_name] = 0.1*np.ones((6,1))
                _lambda = model.u[lambda_var_name]
                _obst_con = -((G@_z - H).T)@_lambda
                GL = (G.T)@_lambda
                _obst_con_abs = norm_2(GL)
                obst_con_var_name = 'obst_con_pt' + str(i+1) + '_other_obst' + str(k+1)
                obst_con_abs_var_name = 'obst_con_abs_pt' + str(i+1) + '_other_obst' + str(k+1)
                mpc.set_nl_cons(obst_con_var_name, _obst_con, ub=0,  soft_constraint=True, penalty_term_cons=100)
                mpc.set_nl_cons(obst_con_abs_var_name, _obst_con_abs, ub=1, soft_constraint=True, penalty_term_cons=100)


        


    mpc.set_objective(mterm=mterm, lterm=lterm)

    
    
#     ############# NEW CONSTRAINTS ##############
#     _lambda1 = model.u['lambda1']
#     _obst_con1 = -((G@_z - H1).T)@_lambda1
#     GL1 = (G.T)@_lambda1
#     _obst_con_abs1 = norm_2(GL1) 
#     #mpc.set_nl_cons('obst_con1', _obst_con1, ub=0,  soft_constraint=True, penalty_term_cons=100)
#     #mpc.set_nl_cons('obst_con_abs1', _obst_con_abs1, ub=1, soft_constraint=True, penalty_term_cons=100)


#     _lambda2 = model.u['lambda2']
#     _obst_con2 = -((G@_z - H2).T)@_lambda2
#     GL2 = (G.T)@_lambda2
#     _obst_con_abs2 = norm_2(GL2) 
#     #mpc.set_nl_cons('obst_con2', _obst_con2, ub=0,  soft_constraint=True, penalty_term_cons=100)
#     #mpc.set_nl_cons('obst_con_abs2', _obst_con_abs2, ub=1, soft_constraint=True, penalty_term_cons=100)
    
    
#     _lambda3 = model.u['lambda3']
#     _obst_con3 = -((G@_z - H3).T)@_lambda3
#     GL3 = (G.T)@_lambda3
#     _obst_con_abs3 = norm_2(GL3) 
#     #mpc.set_nl_cons('obst_con3', _obst_con3, ub=0,  soft_constraint=True, penalty_term_cons=100)
#     #mpc.set_nl_cons('obst_con_abs3', _obst_con_abs3, ub=1, soft_constraint=True, penalty_term_cons=100)
    
    
#     _lambda4 = model.u['lambda4']
#     _obst_con4 = -((G@_z - H4).T)@_lambda4
#     GL4 = (G.T)@_lambda4
#     _obst_con_abs4 = norm_2(GL4) 
#     #mpc.set_nl_cons('obst_con4', _obst_con4, ub=0,  soft_constraint=True, penalty_term_cons=100)
#    # mpc.set_nl_cons('obst_con_abs4', _obst_con_abs4, ub=1, soft_constraint=True, penalty_term_cons=100)
    
    
#     _lambda5 = model.u['lambda5']
#     _obst_con5 = -((G@_z - H5).T)@_lambda5
#     GL5 = (G.T)@_lambda5
#     _obst_con_abs5 = norm_2(GL5) 
#     #mpc.set_nl_cons('obst_con5', _obst_con5, ub=0,  soft_constraint=True, penalty_term_cons=100)
#     #mpc.set_nl_cons('obst_con_abs5', _obst_con_abs5, ub=1, soft_constraint=True, penalty_term_cons=100)
    


    mpc.setup()

    return mpc

