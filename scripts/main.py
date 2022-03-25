#!/home/atharva/BTP/env/bin/python

import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
import time
sys.path.append('../../')
import do_mpc
import time

from std_srvs.srv import Empty
from numpy.lib.function_base import vectorize
import rospy
from std_msgs.msg import String
import numpy as np
import std_msgs.msg
import math
#import tf
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Odometry



from template_model import template_model
from template_mpc import template_mpc
from template_simulator import template_simulator


class multiMPC:
    def __init__(self,mpc,model,simulator,estimator,multi_agent_params):
        self.mpc = mpc
        self.model = model
        self.simulator = simulator    
        self.estimator = estimator
        self.multi_agent_params = multi_agent_params

    def setInitLambda(self):
        lambda0 = 0.1*np.ones((10,1))
        OBSlambda0 = 0.1*np.ones((6,1))
        lambda0[0] = 0.3
        lambda0[3] = 0.3
        lambda0[5] = 0.3
        OBSlambda0[0] = 0.3
        OBSlambda0[3] = 0.3
        OBSlambda0[5] = 0.3
        n_x = self.multi_agent_params['n_x']
        for i in range(n_x):
            for j in (j for j in range(n_x) if j!=i):
                lambda_var_name = 'lambda' + str(i+1) + '_for_agent' + str(j+1)
                self.mpc.u0[lambda_var_name] = lambda0
                self.simulator.u0[lambda_var_name] = lambda0
                self.estimator.u0[lambda_var_name] = lambda0
            n_o = self.multi_agent_params['n_o']
            if n_o!=0:
                for k in range(n_o):
                    lambda_var_name = 'lambda' + str(i+1) + '_for_obst' + str(k+1)
                    self.mpc.u0[lambda_var_name] = OBSlambda0
                    self.simulator.u0[lambda_var_name] = OBSlambda0
                    self.estimator.u0[lambda_var_name] = OBSlambda0

    def setInitX0(self):
        n_x = self.multi_agent_params['n_x'] 
        init_pos = self.multi_agent_params['init_pos']
        x0 = np.zeros((12,1))
        for i in range(n_x):
            var_name = 'x' + str(i+1)
            x0[0] = init_pos[0,i]
            x0[1] = init_pos[1,i]
            x0[2] = init_pos[2,i]
            self.mpc.x0[var_name] = x0
            self.simulator.x0[var_name] = x0
            self.estimator.x0[var_name] = x0

    def setInitU0(self):
        u0 = 0.05*np.ones((4,1))
        n_x = self.multi_agent_params['n_x'] 
        for i in range(n_x):
            var_name = 'u' + str(i+1)
            self.mpc.u0[var_name] = u0
            self.simulator.u0[var_name] = u0
            self.estimator.u0[var_name] = u0






def odometry_callback1(data1):
    global OdomData1
    OdomData1 = data1

def odometry_callback2(data2):
    global OdomData2
    OdomData2 = data2

def odometry_callback3(data3):
    global OdomData3
    OdomData3 = data3

def odometry_callback4(data4):
    global OdomData4
    OdomData4 = data4

def odometry_callback5(data5):
    global OdomData5
    OdomData5 = data5

def odometry_callback6(data6):
    global OdomData6
    OdomData6 = data6
    
def quaternion_to_euler_angle_vectorized1(w, x, y, z):
    ysqr = y * y

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + ysqr)
    X = np.degrees(np.arctan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = np.where(t2>+1.0,+1.0,t2)
    #t2 = +1.0 if t2 > +1.0 else t2

    t2 = np.where(t2<-1.0, -1.0, t2)
    #t2 = -1.0 if t2 < -1.0 else t2
    Y = np.degrees(np.arcsin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (ysqr + z * z)
    Z = np.degrees(np.arctan2(t3, t4))

    return X, Y, Z 


def euler_from_quaternion(w, x, y, z):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians



def mpc():

    """ User settings: """
    show_animation = False
    store_results = False

    """ Problem Declaration """
    n_x = 6 # number of agents
    # n_o = 0 # number of obstacles

    """ Initial Position of Agents """
    init_pos = np.zeros((3,n_x))
    goal_pos = np.zeros((3,n_x))

    # init_pos[:,0] = [OdomData1.pose.pose.position.x,OdomData1.pose.pose.position.y,OdomData1.pose.pose.position.z]
    # goal_pos[:,0] = [0,10,3]

    # init_pos[:,1] = [OdomData2.pose.pose.position.x,OdomData2.pose.pose.position.y,OdomData1.pose.pose.position.z]
    # goal_pos[:,1] = [10,10,3]

    # init_pos[:,2] = [OdomData3.pose.pose.position.x,OdomData3.pose.pose.position.y,OdomData1.pose.pose.position.z]
    # goal_pos[:,2] = [-10,10,3]

    # init_pos[:,3] = [OdomData4.pose.pose.position.x,OdomData4.pose.pose.position.y,OdomData1.pose.pose.position.z]
    # goal_pos[:,3] = [10,0,3]

    # init_pos[:,4] = [OdomData5.pose.pose.position.x,OdomData5.pose.pose.position.y,OdomData1.pose.pose.position.z]
    # goal_pos[:,4] = [-10,0,3]

    # init_pos[:,5] = [OdomData6.pose.pose.position.x,OdomData6.pose.pose.position.y,OdomData1.pose.pose.position.z]
    # goal_pos[:,5] = [0,0,3]


    # init_pos[:,0] = [-10,0,0.08]
    # goal_pos[:,0] = [0,10,3]

    # init_pos[:,1] = [0,0,0.08]
    # goal_pos[:,1] = [10,10,3]

    # init_pos[:,2] = [10,0,0.08]
    # goal_pos[:,2] = [-10,10,3]

    # init_pos[:,3] = [-10,10,0.08]
    # goal_pos[:,3] = [10,0,3]

    # init_pos[:,4] = [0,10,0.08]
    # goal_pos[:,4] = [-10,0,3]

    # init_pos[:,5] = [10,10,0.08]
    # goal_pos[:,5] = [0,0,3]


    # init_pos[:,0] = [-5,8.66,0.08]
    # goal_pos[:,0] = [5,-8.66,5]

    # init_pos[:,1] = [5,8.66,0.08]
    # goal_pos[:,1] = [-5,-8.66,5]

    # init_pos[:,2] = [10,0,0.08]
    # goal_pos[:,2] = [-10,0,5]

    # init_pos[:,3] = [5,-8.66,0.08]
    # goal_pos[:,3] = [-5,8.66,5]

    # init_pos[:,4] = [-5,-8.66,0.08]
    # goal_pos[:,4] = [5,8.66,5]

    # init_pos[:,5] = [-10,0,0.08]
    # goal_pos[:,5] = [10,0,5]

    # init_pos = 0.5*init_pos
    # goal_pos = 0.5*goal_pos



    ## The one with obstacle channel

    init_pos[:,0] = [-4,6,0.08]
    goal_pos[:,0] = [4,-6,3]

    init_pos[:,1] = [0,6,0.08]
    goal_pos[:,1] = [0,-6,3]

    init_pos[:,2] = [4,6,0.08]
    goal_pos[:,2] = [-4,-6,3]

    init_pos[:,3] = [4,-6,0.08]
    goal_pos[:,3] = [-4,6,3]

    init_pos[:,4] = [0,-6,0.08]
    goal_pos[:,4] = [0,6,3]

    init_pos[:,5] = [-4,-6,0.08]
    goal_pos[:,5] = [4,6,3]

  




    # init_pos[:,0] = [0,-10,0.08]
    # goal_pos[:,0] = [0,10,3]

    # init_pos[:,1] = [8.66,-5,0.08]
    # goal_pos[:,1] = [-8.66,5,3]

    # init_pos[:,2] = [8.66,5,0.08]
    # goal_pos[:,2] = [-8.66,-5,3]

    # init_pos[:,3] = [0,10,0.08]
    # goal_pos[:,3] = [0,-10,3]

    # init_pos[:,4] = [-8.66,5,0.08]
    # goal_pos[:,4] = [8.66,-5,3]

    # init_pos[:,5] = [-8.66,-5,0.08]
    # goal_pos[:,5] = [8.66,5,3]



    # init_pos[:,0] = [0,-2.5,0.08]
    # goal_pos[:,0] = [0,2.5,3]

    # init_pos[:,1] = [4.33,-2.5,0.08]
    # goal_pos[:,1] = [-4.33,2.5,3]

    # init_pos[:,2] = [4.33,2.5,0.08]
    # goal_pos[:,2] = [-4.33,-2.5,3]

    # init_pos[:,3] = [0,2.5,0.08]
    # goal_pos[:,3] = [0,-2.5,3]

    # init_pos[:,4] = [-4.33,2.5,0.08]
    # goal_pos[:,4] = [4.33,-2.5,3]

    # init_pos[:,5] = [-4.33,-2.5,0.08]
    # goal_pos[:,5] = [4.33,2.5,3]


    # obstG = np.zeros((6,3))
    # obstG[0,0] = 1
    # obstG[1,0] = -1
    # obstG[2,1] = 1
    # obstG[3,1] = -1
    # obstG[4,2] = 1
    # obstG[5,2] = -1
    
    # obstH = np.ones((6,1))
    # obstH[0,0] = 6
    # obstH[1,0] = -4
    # obstH[2,0] = 1
    # obstH[3,0] = 1
    # obstH[4,0] = 1
    # obstH[5,0] = 1

    # AllObsG = np.array[[obstG]]
    # AllObsH = np.array[[obstH]]

    # if n_o == 0:
    #     AllObsG = 0
    #     AllObsH = 0

    AllObsG = []
    AllObsH = []

    G1 = [[1.,0.,0.],
          [0.,1.,0.],
          [-1.,0.,0.],
          [0.,-1.,0.],
          [0.,0.,1.],
          [0.,0.,-1.]]
    AllObsG.append(G1)
    H1 = [[-3],
          [4],
          [15],
          [4],
          [10],
          [10]]
    AllObsH.append(H1)


    G2 = [[1,0,0],
          [0,1,0],
          [-1,0,0],
          [0,-1,0],
          [0,0,1],
          [0,0,-1]]
    AllObsG.append(G2)
    H2 = [[15],
          [4],
          [-3],
          [4],
          [10],
          [10]]
    AllObsH.append(H2)

    n_o = len(AllObsG)  # number of obstacles
    print('OBS LEN: '+str(n_o))


    multi_agent_params = {
        'n_x': n_x,
        'n_o': n_o,
        'init_pos': init_pos,
        'goal_pos': goal_pos,
        'obst_G': AllObsG,
        'obst_H': AllObsH,
        'n_horizon': 17     
    }

    """
    Get configured do-mpc modules:
    """
    model = template_model(multi_agent_params)
    mpc = template_mpc(model,multi_agent_params)
    simulator = template_simulator(model,mpc,multi_agent_params)
    estimator = do_mpc.estimator.StateFeedback(model)

    """
    Initialise MultiMPC Class
    """
    _multiMPC = multiMPC(mpc,model,simulator,estimator,multi_agent_params) 


    """
    Set initial state
    """
    # np.random.seed(99)

    # e = np.zeros([model.n_x,1])
    # #e[2] = 5
    # x0 = e # Values between +3 and +3 for all states
    # mpc.x0 = x0
    # simulator.x0 = x0
    # estimator.x0 = x0
    # lambda0 = 0.1*np.ones((6,1))
    # lambda0[0] = 0.3
    # lambda0[3] = 0.3
    # u0 = 0*np.ones((4,1))
    # mpc.u0['u'] = u0
    # simulator.u0['u'] = u0
    # estimator.u0['u'] = u0
    

    #############################################
    _multiMPC.setInitLambda()
    _multiMPC.setInitX0()
    _multiMPC.setInitU0()
    



    # mpc.u0['lambda'] = lambda0
    
    # simulator.u0['lambda'] = lambda0
    
    # estimator.u0['lambda'] = lambda0
    


    # mpc.u0['lambda1'] = lambda0
    # simulator.u0['lambda1'] = lambda0
    # estimator.u0['lambda1'] = lambda0

    # mpc.u0['lambda2'] = lambda0
    # simulator.u0['lambda2'] = lambda0
    # estimator.u0['lambda2'] = lambda0
        
    # mpc.u0['lambda3'] = lambda0
    # simulator.u0['lambda3'] = lambda0
    # estimator.u0['lambda3'] = lambda0
        
    # mpc.u0['lambda4'] = lambda0
    # simulator.u0['lambda4'] = lambda0
    # estimator.u0['lambda4'] = lambda0
    
    # mpc.u0['lambda5'] = lambda0
    # simulator.u0['lambda5'] = lambda0
    # estimator.u0['lambda5'] = lambda0
    
    
    # Use initial state to set the initial guess.
    mpc.set_initial_guess()

    """
    Setup graphic:
    """

    fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data)
    plt.ion()
    
    

    # """
    # Run MPC main loop:
    # """

    # for k in range(200):
    #     u0 = mpc.make_step(x0)
    #     y_next = simulator.make_step(u0)
    #     x0 = estimator.make_step(y_next)
    #     print(x0[2])

    #     if show_animation:
    #         graphics.plot_results(t_ind=k)
    #         graphics.plot_predictions(t_ind=k)
    #         graphics.reset_axes()
    #         plt.show()
    #         plt.pause(0.01)

    # input('Press any key to exit.')
    
    """
    ROS Subscriber and Publisher 
    """
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    firefly_command_publisher_1 = rospy.Publisher('/firefly1/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    firefly_command_publisher_2 = rospy.Publisher('/firefly2/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    firefly_command_publisher_3 = rospy.Publisher('/firefly3/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    firefly_command_publisher_4 = rospy.Publisher('/firefly4/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    firefly_command_publisher_5 = rospy.Publisher('/firefly5/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    firefly_command_publisher_6 = rospy.Publisher('/firefly6/command/trajectory', MultiDOFJointTrajectory, queue_size=10)
    agent1_command_publisher = rospy.Publisher('/agent1',Pose, queue_size=10)
    agent2_command_publisher = rospy.Publisher('/agent2',Pose, queue_size=10)
    agent3_command_publisher = rospy.Publisher('/agent3',Pose, queue_size=10)
    vel1_command_publisher = rospy.Publisher('/velocity1',Pose, queue_size=10)
    vel2_command_publisher = rospy.Publisher('/velocity2',Pose, queue_size=10)
    #rospy.Subscriber('/firefly/odometry_sensor1/odometry', Odometry, odometry_callback)

    pos1 = Pose()
    pos2 = Pose()
    pos3 = Pose()

    vel1 = Pose()
    vel2 = Pose()


    traj1 = MultiDOFJointTrajectory()

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time()
    header.frame_id = 'frame'
    traj1.joint_names.append('base_link')
    traj1.header=header
    transforms =Transform()
    velocities =Twist()
    accelerations=Twist()


    traj2 = MultiDOFJointTrajectory()
    traj2.joint_names.append('base_link')
    traj2.header=header

    traj3 = MultiDOFJointTrajectory()
    traj3.joint_names.append('base_link')
    traj3.header=header

    traj4 = MultiDOFJointTrajectory()
    traj4.joint_names.append('base_link')
    traj4.header=header

    traj5 = MultiDOFJointTrajectory()
    traj5.joint_names.append('base_link')
    traj5.header=header

    traj6 = MultiDOFJointTrajectory()
    traj6.joint_names.append('base_link')
    traj6.header=header
    
    x_log = np.zeros((1500,18))
    x0 = _multiMPC.mpc.x0
    # print(x0)
    # print(x0.shape)

    u0 = _multiMPC.mpc.u0
    # print(u0)
    # print(u0)
        
    """
    Run MPC ROS Loop
    """
    k = 0
    
    while not rospy.is_shutdown():
        
        # unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        # rospy.sleep(0.01)
        # pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        
        #print(OdomData)
        start_time = time.time()
        
        W = OdomData1.pose.pose.orientation.w
        X = OdomData1.pose.pose.orientation.x
        Y = OdomData1.pose.pose.orientation.y
        Z = OdomData1.pose.pose.orientation.z
        
        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        x1 = x0['x1']

        x1[0] = OdomData1.pose.pose.position.x
        x1[1] = OdomData1.pose.pose.position.y
        x1[2] = OdomData1.pose.pose.position.z
        x1[3] = OdomData1.twist.twist.linear.x
        x1[4] = OdomData1.twist.twist.linear.y
        x1[5] = OdomData1.twist.twist.linear.z
        x1[6] = phi
        x1[7] = theta
        x1[8] = psi
        x1[9] = OdomData1.twist.twist.angular.x
        x1[10] = OdomData1.twist.twist.angular.y
        x1[11] = OdomData1.twist.twist.angular.z

        

######################################################################




        W = OdomData2.pose.pose.orientation.w
        X = OdomData2.pose.pose.orientation.x
        Y = OdomData2.pose.pose.orientation.y
        Z = OdomData2.pose.pose.orientation.z
        
        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        x2 = x0['x2']

        x2[0] = OdomData2.pose.pose.position.x
        x2[1] = OdomData2.pose.pose.position.y
        x2[2] = OdomData2.pose.pose.position.z
        x2[3] = OdomData2.twist.twist.linear.x
        x2[4] = OdomData2.twist.twist.linear.y
        x2[5] = OdomData2.twist.twist.linear.z
        x2[6] = phi
        x2[7] = theta
        x2[8] = psi
        x2[9] = OdomData2.twist.twist.angular.x
        x2[10] = OdomData2.twist.twist.angular.y
        x2[11] = OdomData2.twist.twist.angular.z


#################################################################

        W = OdomData3.pose.pose.orientation.w
        X = OdomData3.pose.pose.orientation.x
        Y = OdomData3.pose.pose.orientation.y
        Z = OdomData3.pose.pose.orientation.z
        
        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        x3 = x0['x3']

        x3[0] = OdomData3.pose.pose.position.x
        x3[1] = OdomData3.pose.pose.position.y
        x3[2] = OdomData3.pose.pose.position.z
        x3[3] = OdomData3.twist.twist.linear.x
        x3[4] = OdomData3.twist.twist.linear.y
        x3[5] = OdomData3.twist.twist.linear.z
        x3[6] = phi
        x3[7] = theta
        x3[8] = psi
        x3[9] = OdomData3.twist.twist.angular.x
        x3[10] = OdomData3.twist.twist.angular.y
        x3[11] = OdomData3.twist.twist.angular.z

#############################################################


        W = OdomData4.pose.pose.orientation.w
        X = OdomData4.pose.pose.orientation.x
        Y = OdomData4.pose.pose.orientation.y
        Z = OdomData4.pose.pose.orientation.z
        
        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        x4 = x0['x4']

        x4[0] = OdomData4.pose.pose.position.x
        x4[1] = OdomData4.pose.pose.position.y
        x4[2] = OdomData4.pose.pose.position.z
        x4[3] = OdomData4.twist.twist.linear.x
        x4[4] = OdomData4.twist.twist.linear.y
        x4[5] = OdomData4.twist.twist.linear.z
        x4[6] = phi
        x4[7] = theta
        x4[8] = psi
        x4[9] = OdomData4.twist.twist.angular.x
        x4[10] = OdomData4.twist.twist.angular.y
        x4[11] = OdomData4.twist.twist.angular.z

#############################################################


        W = OdomData5.pose.pose.orientation.w
        X = OdomData5.pose.pose.orientation.x
        Y = OdomData5.pose.pose.orientation.y
        Z = OdomData5.pose.pose.orientation.z
        
        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        x5 = x0['x5']

        x5[0] = OdomData5.pose.pose.position.x
        x5[1] = OdomData5.pose.pose.position.y
        x5[2] = OdomData5.pose.pose.position.z
        x5[3] = OdomData5.twist.twist.linear.x
        x5[4] = OdomData5.twist.twist.linear.y
        x5[5] = OdomData5.twist.twist.linear.z
        x5[6] = phi
        x5[7] = theta
        x5[8] = psi
        x5[9] = OdomData5.twist.twist.angular.x
        x5[10] = OdomData5.twist.twist.angular.y
        x5[11] = OdomData5.twist.twist.angular.z

# #############################################################


        W = OdomData6.pose.pose.orientation.w
        X = OdomData6.pose.pose.orientation.x
        Y = OdomData6.pose.pose.orientation.y
        Z = OdomData6.pose.pose.orientation.z
        
        phi,theta,psi = quaternion_to_euler_angle_vectorized1(W,X,Y,Z)
        phi,theta,psi = euler_from_quaternion(W,X,Y,Z)
        
        x6 = x0['x6']

        x6[0] = OdomData6.pose.pose.position.x
        x6[1] = OdomData6.pose.pose.position.y
        x6[2] = OdomData6.pose.pose.position.z
        x6[3] = OdomData6.twist.twist.linear.x
        x6[4] = OdomData6.twist.twist.linear.y
        x6[5] = OdomData6.twist.twist.linear.z
        x6[6] = phi
        x6[7] = theta
        x6[8] = psi
        x6[9] = OdomData6.twist.twist.angular.x
        x6[10] = OdomData6.twist.twist.angular.y
        x6[11] = OdomData6.twist.twist.angular.z

############################################################################

        x0['x1'] = x1
        x0['x2'] = x2
        x0['x3'] = x3
        x0['x4'] = x4
        x0['x5'] = x5
        x0['x6'] = x6

        x_log[k,0] = OdomData1.pose.pose.position.x
        x_log[k,1] = OdomData1.pose.pose.position.y
        x_log[k,2] = OdomData1.pose.pose.position.z

        x_log[k,3] = OdomData2.pose.pose.position.x
        x_log[k,4] = OdomData2.pose.pose.position.y
        x_log[k,5] = OdomData2.pose.pose.position.z

        x_log[k,6] = OdomData3.pose.pose.position.x
        x_log[k,7] = OdomData3.pose.pose.position.y
        x_log[k,8] = OdomData3.pose.pose.position.z

        x_log[k,9] = OdomData4.pose.pose.position.x
        x_log[k,10] = OdomData4.pose.pose.position.y
        x_log[k,11] = OdomData4.pose.pose.position.z

        x_log[k,12] = OdomData5.pose.pose.position.x
        x_log[k,13] = OdomData5.pose.pose.position.y
        x_log[k,14] = OdomData5.pose.pose.position.z

        x_log[k,15] = OdomData6.pose.pose.position.x
        x_log[k,16] = OdomData6.pose.pose.position.y
        x_log[k,17] = OdomData6.pose.pose.position.z

        # x0[12] = OdomData2.pose.pose.position.x
        # x0[13] = OdomData2.pose.pose.position.y
        # x0[14] = OdomData2.pose.pose.position.z
        # x0[15] = OdomData2.twist.twist.linear.x
        # x0[16] = OdomData2.twist.twist.linear.y
        # x0[17] = OdomData2.twist.twist.linear.z
        # x0[18] = phi
        # x0[19] = theta
        # x0[20] = psi
        # x0[21] = OdomData2.twist.twist.angular.x
        # x0[22] = OdomData2.twist.twist.angular.y
        # x0[23] = OdomData2.twist.twist.angular.z
        
    
        
        _multiMPC.mpc.set_initial_guess()
        
        u0 = _multiMPC.mpc.make_step(x0)
        
#################################################################################

        # for i in range(n_x):
        #     x_var_name = 'x' + str(i+1)
        #     u_var_name = 'u' + str(i+1)
        #     _x = model.x[x_var_name]
        #     _u = model.u[u_var_name]
        #     _z = np.array([[_x[0]],
        #             [_x[1]],
        #             [_x[2]]])

        #     for j in (j for j in range(n_x) if j!=i):
        #         lambda_var_name = 'lambda' + str(i+1) + '_for_agent' + str(j+1)
        #         G_var_name = 'G' + str(j+1)
        #         H_var_name = 'H' + str(j+1)
        #         _lambda = model.u[lambda_var_name]
        #         G = model.tvp[G_var_name]
        #         H = model.tvp[H_var_name]
        #         _obst_con = -((G@_z - H).T)@_lambda
        #         GL = (G.T)@_lambda
        #         _obst_con_abs = norm_2(GL)
        #         print(lambda_var_name)
        #         print(G)


        lb_bound_violation = mpc.opt_x_num.cat <= mpc.lb_opt_x
        ub_bound_violation = mpc.opt_x_num.cat <= mpc.ub_opt_x

        # opt_labels = mpc.opt_x.labels()
        # labels_lb_viol =np.array(opt_labels)[np.where(lb_bound_violation)[0]]
        # labels_ub_viol =np.array(opt_labels)[np.where(lb_bound_violation)[0]]

        print(lb_bound_violation)
        print(ub_bound_violation)
        # print(labels_lb_viol)
        # print(labels_ub_viol)

        print(k)
        print(x0['x1'].T)
        print(x0['x2'].T)
        print(x0['x3'].T)
        print(x0['x4'].T)
        print(x0['x5'].T)
        print(x0['x6'].T)





#################################################################################




        # print(u0.T)
        
        # # mpc.x0 = x0
        # # simulator.x0 = x0
        # # estimator.x0 = x0
        # # mpc.u0 = u0
        # # simulator.u0 = u0
        # # estimator.u0 = u0
        # mpc.set_initial_guess()
        
        
        
        # lambda0 = 0.1*np.ones((6,1))
        # lambda0[0] = 0.3
        # lambda0[3] = 0.3
        # _u0 = 0*np.ones((4,1))
        # mpc.u0['lambda'] = lambda0
        # mpc.u0['u'] = _u0
        # simulator.u0['lambda'] = lambda0
        # simulator.u0['u'] = _u0
        # estimator.u0['lambda'] = lambda0
        # estimator.u0['u'] = _u0
        
        # mpc.u0['lambda1'] = lambda0
        # simulator.u0['lambda1'] = lambda0
        # estimator.u0['lambda1'] = lambda0

        # mpc.u0['lambda2'] = lambda0
        # simulator.u0['lambda2'] = lambda0
        # estimator.u0['lambda2'] = lambda0
        
        # mpc.u0['lambda3'] = lambda0
        # simulator.u0['lambda3'] = lambda0
        # estimator.u0['lambda3'] = lambda0
        
        # mpc.u0['lambda4'] = lambda0
        # simulator.u0['lambda4'] = lambda0
        # estimator.u0['lambda4'] = lambda0
        
        # mpc.u0['lambda5'] = lambda0
        # simulator.u0['lambda5'] = lambda0
        # estimator.u0['lambda5'] = lambda0
        
        
        
        # y_next = simulator.make_step(u0)
        # x0 = estimator.make_step(y_next)
        _multiMPC.mpc.x0 = x0
        _multiMPC.simulator.x0 = x0
        _multiMPC.estimator.x0 = x0
        _multiMPC.mpc.u0 = u0
        _multiMPC.simulator.u0 = u0
        _multiMPC.estimator.u0 = u0

        u__ = _multiMPC.mpc.u0
        # print(u__)
        # print(u__.shape)
        # print(u0[0])
        # print(u__['u1'])

        # print(x0)



        # """ Publish Position Messages """

        # # _X1 = x0[0]
        # pos1.position.x = x0[0]
        # pos1.position.y = x0[1]
        # pos1.position.z = x0[2]
        # agent1_command_publisher.publish(pos1)

        # # _X2 = x0['x2']
        # pos2.position.x = x0[12]
        # pos2.position.y = x0[13]
        # pos2.position.z = x0[14]
        # agent2_command_publisher.publish(pos2)

        # vel1.position.x = x0[3]
        # vel1.position.y = x0[4]
        # vel1.position.z = x0[5]
        # vel1_command_publisher.publish(vel1)

        # vel2.position.x = x0[15]
        # vel2.position.y = x0[16]
        # vel2.position.z = x0[17]
        # vel2_command_publisher.publish(vel2)

        
        # # _X3 = x0['x3']
        # pos3.position.x = x0[24]
        # pos3.position.y = x0[25]
        # pos3.position.z = x0[26]
        # agent3_command_publisher.publish(pos3)
        
        
        
        
        if show_animation:
            graphics.plot_results(t_ind=k)
            graphics.plot_predictions(t_ind=k)
            graphics.reset_axes()
            plt.show()
            plt.pause(0.01)
            
            
            
        
        # accelerations.linear.x = 0
        # accelerations.linear.y = 0
        # accelerations.linear.z = u0[0]
        
        # accelerations.angular.x = u0[1]
        # accelerations.angular.y = u0[2]
        # accelerations.angular.z = u0[3]
        
        # point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())

        # traj.points.append(point)
        # x_log[0,k] = x0[0]
        # x_log[1,k] = x0[1]
        # x_log[2,k] = x0[2]
        #rate.sleep()
        # firefly_command_publisher.publish(traj)    


        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[0]
        
        accelerations.angular.x = u0[1]
        accelerations.angular.y = u0[2]
        accelerations.angular.z = u0[3]
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        traj1.points.append(point)
        firefly_command_publisher_1.publish(traj1)
        print(accelerations.linear.z,accelerations.angular.x,accelerations.angular.y,accelerations.angular.z)



        temp = 4 + (n_x-1)*10 + (n_o)*6

        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[temp]
        
        accelerations.angular.x = u0[temp+1]
        accelerations.angular.y = u0[temp+2]
        accelerations.angular.z = u0[temp+3]
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        traj2.points.append(point)
        firefly_command_publisher_2.publish(traj2)
        print(accelerations.linear.z,accelerations.angular.x,accelerations.angular.y,accelerations.angular.z)





        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[2*temp]
        
        accelerations.angular.x = u0[2*temp+1]
        accelerations.angular.y = u0[2*temp+2]
        accelerations.angular.z = u0[2*temp+3]
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        traj3.points.append(point)
        firefly_command_publisher_3.publish(traj3)
        print(accelerations.linear.z,accelerations.angular.x,accelerations.angular.y,accelerations.angular.z)





        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[3*temp]
        
        accelerations.angular.x = u0[3*temp+1]
        accelerations.angular.y = u0[3*temp+2]
        accelerations.angular.z = u0[3*temp+3]
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        traj4.points.append(point)
        firefly_command_publisher_4.publish(traj4)
        print(accelerations.linear.z,accelerations.angular.x,accelerations.angular.y,accelerations.angular.z)






        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[4*temp]
        
        accelerations.angular.x = u0[4*temp+1]
        accelerations.angular.y = u0[4*temp+2]
        accelerations.angular.z = u0[4*temp+3]
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        traj5.points.append(point)
        firefly_command_publisher_5.publish(traj5)
        print(accelerations.linear.z,accelerations.angular.x,accelerations.angular.y,accelerations.angular.z)





        accelerations.linear.x = 0
        accelerations.linear.y = 0
        accelerations.linear.z = u0[5*temp]
        
        accelerations.angular.x = u0[5*temp+1]
        accelerations.angular.y = u0[5*temp+2]
        accelerations.angular.z = u0[5*temp+3]
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accelerations],rospy.Time.now())
        traj6.points.append(point)
        firefly_command_publisher_6.publish(traj6)
        print(accelerations.linear.z,accelerations.angular.x,accelerations.angular.y,accelerations.angular.z)



        end_time = time.time()
        time_taken = (end_time - start_time)#/(1e9)
        print("time_taken: ",time_taken)
        k = k + 1
        #rate.sleep()
        #rospy.sleep()
        # if k == 1000:
        #     break
        
        

            
    
    # Store results:
    if store_results:
        do_mpc.data.save_results([mpc, simulator], 'oscillating_masses')
        
    np.save('six_agent', x_log)






################################################################################################
################################################################################################

if __name__ == '__main__':
    try:
        rospy.Subscriber('/firefly1/odometry_sensor1/odometry', Odometry, odometry_callback1)
        rospy.Subscriber('/firefly2/odometry_sensor1/odometry', Odometry, odometry_callback2)
        rospy.Subscriber('/firefly3/odometry_sensor1/odometry', Odometry, odometry_callback3)
        rospy.Subscriber('/firefly4/odometry_sensor1/odometry', Odometry, odometry_callback4)
        rospy.Subscriber('/firefly5/odometry_sensor1/odometry', Odometry, odometry_callback5)
        rospy.Subscriber('/firefly6/odometry_sensor1/odometry', Odometry, odometry_callback6)
        # mpc_takeoff()
        #time.sleep(5)
        mpc()
    except rospy.ROSInterruptException:
        pass
