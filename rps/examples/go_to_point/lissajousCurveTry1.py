import rps.robotarium as robotarium
from rps.utilities.transformations import *
from rps.utilities.barrier_certificates import *
from rps.utilities.misc import *
from rps.utilities.controllers import *

import numpy as np
import time

# Instantiate Robotarium object
N = 2
Pi=np.pi
initial_conditions = generate_initial_conditions(N)#(x[N];y[N];phi[N])

r = robotarium.Robotarium(number_of_robots=N, show_figure=True, initial_conditions=initial_conditions, sim_in_real_time=False)

t=0
xpo=[0,-0.5]
ypo=[0, 0]
t=str(t)
# Define goal points by removing orientation from poses
goal_points = np.array(np.mat('0  -1 ; 0  0; 0  0'))


# Create single integrator position controller
single_integrator_position_controller = create_si_position_controller()

# Create barrier certificates to avoid collision
#si_barrier_cert = create_single_integrator_barrier_certificate()
si_barrier_cert = create_single_integrator_barrier_certificate_with_boundary()

_, uni_to_si_states = create_si_to_uni_mapping()

# Create mapping from single integrator velocity commands to unicycle velocity commands
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion()

# define x initially
x = r.get_poses()
x_si = uni_to_si_states(x)
r.step()

# While the number of robots at the required poses is less
# than N...
while (float(t)<5000):
    xpo[0]=str(xpo[0])
    xpo[1]=str(xpo[1])
    ypo[0]=str(ypo[0])
    ypo[1]=str(ypo[1])
    goal_points = np.array(np.mat(xpo[0]+' '+xpo[1]+' ;'+ypo[0]+' '+ypo[1]+';0 0'))
    while (np.size(at_pose(np.vstack((x_si,x[2,:])), goal_points, rotation_error=200)) != N):

        # Get poses of agents
        x = r.get_poses()
        x_si = uni_to_si_states(x)

        # Create single-integrator control inputs
        dxi = single_integrator_position_controller(x_si, goal_points[:2][:])
 
        # Create safe control inputs (i.e., no collisions)
        dxi = si_barrier_cert(dxi, x_si)

        # Transform single integrator velocity commands to unicycle
        dxu = si_to_uni_dyn(dxi, x)

        # Set the velocities by mapping the single-integrator inputs to unciycle inputs
        r.set_velocities(np.arange(N), dxu)
        # Iterate the simulation
        r.step()

    #Call at end of script to print debug information and for your script to run on the Robotarium server properly
    t=float(t)+1
    thao=t/100
    Pi=np.pi
    a=[1,3,2,3]
    xpo[0]=0.2*np.sin(a[0]*thao)
    ypo[0]=0.2*np.sin(a[1]*thao+Pi/2)
    xpo[1]=0.2*np.sin(a[2]*thao)-0.6
    ypo[1]=0.2*np.sin(a[3]*thao+Pi/2)
    plt.plot(xpo[0],ypo[0],'ro')
    print(t)
    #xpo=(np.cos(a*thao)-np.power(np.cos(b*thao),3))/2
    #ypo=(np.sin(c*thao)-np.power(np.sin(d*thao),3))/2
    #xpo=(16*np.power(np.sin(thao),3))/20
    #ypo=(13*np.cos(thao)-5*np.cos(2*thao)-2*np.cos(3*thao)-np.cos(4*thao))/20
r.call_at_scripts_end()
