# Cameron J. Calv 
# ECES T580 Applied Robotics
# Programming Assignment 4
#   Determine torque characteristics of the manipulator using Lagrangian Dynamics
import math
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

#Matrix multiplication function developped during my Numerical Methods class with Dr. Peters
# with slight modification to make it fit for this application
def mmult(matrix_A, matrix_B):
    product = [] 
    n = len(matrix_A)
    m = len(matrix_A[0])
    try:
        l = len(matrix_B[0])
    except TypeError:
        l = 1
    if (m == len(matrix_B)):
        product = []
        for r in range(n):
            product.append([])
            for c in range(l):
                product[r].append([])
        for i in range(n):
            for j in range(l):
                sum = 0
                for k in range(m):
                    sum += matrix_A[i][k]*matrix_B[k][j]
                product[i][j] = sum
    return product

# Generates X and Y values for an arbitray line according to Ax+B=y
def linear_gen(a, b, x_step=0.1, x_init=0, x_final=1):
    xs = []
    ys = []
    if x_init == x_final or x_step == 0:
        return xs, ys
    if x_step < 0:
        x_step = -1*x_step
    if x_final < x_init:
        temp = x_final
        x_final = x_init
        x_init = temp
    this_x = x_init
    for i in range(math.ceil((x_final-x_init)/x_step)+1):
        xs.append(this_x)
        ys.append(a*this_x+b)
        this_x = this_x + x_step
    return xs, ys

# Generates X and Y values for an arbitrary ellipse according to ((x-x_0)/a)^2+((y-y_0)/b)^2=r^2
def ellipse_gen(a=1, b=1, r=1, center_x=0, center_y=0, x_step=0.1, x_init=0, x_final=1):
    xs = []
    ys = []
    if x_init == x_final or x_step == 0 or a == 0 or b == 0 or r == 0:
        return xs, ys
    if x_step < 0:
        x_step = -1*x_step
    if x_final < x_init:
        temp = x_final
        x_final = x_init
        x_init = temp
    # this_x = x_init
    for i in range(math.ceil((x_final-x_init)/x_step)+1):
        this_x = round(i*x_step + x_init, 5)
        med = (a**2)*(b**2)*((a**2)*(r**2)-(center_x**2)+2*center_x*this_x-(this_x**2))
        if med < 0:
            continue
        xs.append(this_x)
        ys.append(((a**2)*center_y-math.sqrt(med))/(a**2))
        xs.append(this_x)
        ys.append(((a**2)*center_y+math.sqrt(med))/(a**2))
        # this_x = this_x + x_step
    return xs, ys

#Determines the link angles of a two-link manipulator for an array of end-effector positions
def two_link_inverse_kinematics(x_vals, y_vals, length_1, length_2, elbow_up=True):
    #Set epsilon value for 'close enough' approximations
    epsilon = 1e-10
    theta_1s = []
    theta_2s = []
    if not len(x_vals)==len(y_vals):
        return theta_1s, theta_2s
    if elbow_up:
        sigma = 1
    else:
        sigma = -1

    #For every test point (TP), determine if it is reachable
    # and if it is, determine the angles
    for i in range(len(x_vals)):
        this_x = x_vals[i]
        this_y = y_vals[i]
        med_root = math.sqrt((this_x**2)+(this_y**2))

        #Is the TP unreachable (out of manipulator's range)
        if (med_root > (length_1+length_2)):
            theta_1s.append(False)
            theta_2s.append(False)
            continue

        #Solve for nonlinear equation
        gamma = math.atan2((-1)*this_y/med_root,(-1)*this_x/med_root)
        med_cos = ((this_x**2)+(this_y**2)+(length_1**2)-(length_2**2))/(-2*length_1*med_root)
        if med_cos > 1:
            med_cos = 1
        elif med_cos < -1:
            med_cos = -1

        #Solve for both Theta_1s
        this_theta_1 = gamma + sigma*math.acos(med_cos)
        this_theta_1_opp = gamma + (-1*sigma)*math.acos(med_cos)

        #Check all combinations of Theta_1s to find the real solution
        found = False
        for possible_theta_1_first in [this_theta_1, this_theta_1_opp]:
            if found:
                break
            for possible_theta_1_second in [this_theta_1, this_theta_1_opp]:
                med_cos = (this_x - length_1*math.cos(possible_theta_1_first))/length_2
                med_sin = (this_y - length_1*math.sin(possible_theta_1_second))/length_2
                this_theta_2 = math.atan2(med_sin, med_cos)-this_theta_1
                anticipated_x_first = length_1*math.cos(possible_theta_1_first)+\
                    length_2*math.cos(possible_theta_1_first+this_theta_2)
                anticipated_y_first = length_1*math.sin(possible_theta_1_first)+\
                    length_2*math.sin(possible_theta_1_first+this_theta_2)
                anticipated_x_second = length_1*math.cos(possible_theta_1_second)+\
                    length_2*math.cos(possible_theta_1_second+this_theta_2)
                anticipated_y_second = length_1*math.sin(possible_theta_1_second)+\
                    length_2*math.sin(possible_theta_1_second+this_theta_2)
                if (abs(anticipated_x_first-this_x)<epsilon) and (abs(anticipated_y_first-this_y)<epsilon):
                    theta_1s.append(possible_theta_1_first)
                    theta_2s.append(this_theta_2)
                    found = True
                    break
                if (abs(anticipated_x_second-this_x)<epsilon) and (abs(anticipated_y_second-this_y)<epsilon):
                    theta_1s.append(possible_theta_1_second)
                    theta_2s.append(this_theta_2)
                    found = True
                    break
        if not found:
            theta_1s.append(False)
            theta_2s.append(False)
    return theta_1s, theta_2s

#Determines the link angles of a two-link manipulator for a given velocity and position profile using Jacobian matrices
def two_link_jacobian_kinematics(time, pos_prof, vel_prof, length_1, length_2, sample_time=0.05, elbow_up=True):
    theta_1s = []
    theta_2s = []
    #Emulate encoder's reading initial value of joint angles
    theta_1_old, theta_2_old = two_link_inverse_kinematics([pos_prof[0][0]], [pos_prof[1][0]],\
        length_1, length_2, elbow_up)
    theta_1_old = theta_1_old[0]
    theta_2_old = theta_2_old[0]
    theta_1s.append(theta_1_old)
    theta_2s.append(theta_2_old)
    for i in range(len(time)-1):
        det = (length_1*length_2*math.sin(theta_2_old))
        # jacobian = [\
        #     [-1*length_1*math.sin(theta_1_old)-length_2*math.sin(theta_1_old+theta_2_old),\
        #     -1*length_2*math.sin(theta_1_old+theta_2_old)],\
        #     [length_1*math.cos(theta_1_old)+length_2*math.cos(theta_1_old+theta_2_old),\
        #     length_2*math.cos(theta_1_old+theta_2_old)]]
        inv_jacobian = [\
            [(length_2*math.cos(theta_1_old+theta_2_old))/det,\
            (length_2*math.sin(theta_1_old+theta_2_old))/det],\
            [-1*(length_1*math.cos(theta_1_old)+length_2*math.cos(theta_1_old+theta_2_old))/det,\
            -1*(length_1*math.sin(theta_1_old)+length_2*math.sin(theta_1_old+theta_2_old))/det]]
        p_dot = [[vel_prof[0][i]], [vel_prof[1][i]]]
        q_dot = mmult(inv_jacobian, p_dot)
        delta_theta_1 = sample_time*q_dot[0][0]
        delta_theta_2 = sample_time*q_dot[1][0]
        theta_1s.append(theta_1_old+delta_theta_1)
        theta_2s.append(theta_2_old+delta_theta_2)
        theta_1_old = theta_1s[-1]
        theta_2_old = theta_2s[-1]
    return theta_1s, theta_2s

#Takes position vectors and a time vector to extrapolate a velocity vector
# Note: assumes zero end velocity
# Note: not physically accurate
def position_to_velocity(time, positions_x, positions_y):
    if not (len(positions_x) == len(positions_y) == len(time)):
        print("Please provide equal sized vectors!")
        return [False], [False]
    v_end = 0
    v1s = []
    v2s = []
    for idx in range(len(time)-1):
        v1s.append((positions_x[idx+1]-positions_x[idx])/(time[idx+1]-time[idx]))
        v2s.append((positions_y[idx+1]-positions_y[idx])/(time[idx+1]-time[idx]))
    v1s.append(v_end)
    v2s.append(v_end)
    return [v1s, v2s]

#Takes position vectors and a time vector to extrapolate a velocity vector (Note, not physically accurate)
# Note: assumes zero end acceleration
# Note: not physically accurate
def position_to_acceleration(time, positions_x, positions_y):
    if not (len(positions_x) == len(positions_y) == len(time)):
        print("Please provide equal sized vectors!")
        return [False], [False]
    [velocities_1, velocities_2] = position_to_velocity(time, positions_x, positions_y)
    accel_end = 0
    a1s = []
    a2s = []
    for idx in range(len(time)-1):
        a1s.append((velocities_1[idx+1]-velocities_1[idx])/(time[idx+1]-time[idx]))
        a2s.append((velocities_2[idx+1]-velocities_2[idx])/(time[idx+1]-time[idx]))
    a1s.append(accel_end)
    a2s.append(accel_end)
    return [a1s, a2s]

#Generates 2 plots: 
#   Top Plots (Plots 1): Showing RR manipulator in physical space with 2 links extending to test pionts
#   Bottom Plots (Plots 2) Creating plots showcasing theta_1/2 vs x/y for elbow up and down positions
#   Option: inverse_kinematics for True, jacobian for False
def plot_kinematic_figures(tp_xs, tp_ys, length_1, length_2, origin, figure_num=1, inverse_kinematics=True, \
    sample_time=0.05, total_time=1.8):
    #Set up plot values
    fig = plt.figure(figure_num)
    subplots = [242, 245, 246, 243, 247, 248]
    titles = ['Elbow Up : Space', r'Elbow Up : X, Y and $\theta_1$', r'Elbow Up : X, Y and $\theta_2$',
        'Elbow Down : Space', r'Elbow Down : X, Y and $\theta_1$', r'Elbow Down : X, Y and $\theta_2$']
    xlabels = ['x', 'x,y', 'x,y', 'x', 'x,y', 'x,y']
    ylabels = ['y', r'$\theta_1$ (deg)', r'$\theta_2$ (deg)', 'y', r'$\theta_1$ (deg)', r'$\theta_2$ (deg)']
    iter = 0
    for elbow in [True, False]:
        if inverse_kinematics:
            theta_1s, theta_2s = two_link_inverse_kinematics(tp_xs, tp_ys, length_1, length_2, elbow_up=elbow)
        else:
            time_array, pos_xy, vel_xy, acc_xy = trap_profile_gen_test_points([tp_xs[0], tp_ys[0]],\
                 [tp_xs[-1], tp_ys[-1]], total_time, sample_time)
            theta_1s, theta_2s = two_link_jacobian_kinematics(time_array, pos_xy, vel_xy,\
                link_lengths, link_lengths, sample_time=sample_time, elbow_up=elbow)            
        
        #Remove any areas the manipulator cannot reach
        relevant_xs = []
        relevant_theta_1s = []
        relevant_ys = []
        relevant_theta_2s = []
        enum = 0
        for theta in theta_1s:
            if theta is not False:
                relevant_xs.append(tp_xs[enum])
                relevant_theta_1s.append(theta)
            enum = enum + 1
        enum = 0
        for theta in theta_2s:
            if theta is not False:
                relevant_ys.append(tp_ys[enum])
                relevant_theta_2s.append(theta)
            enum = enum + 1
        theta_1s = relevant_theta_1s
        theta_2s = relevant_theta_2s

        # Plot physical space drawings test points
        axis = fig.add_subplot(subplots[iter])
        axis.set_aspect('equal', adjustable='box')
        plt.scatter(relevant_xs, relevant_ys, s=8, c='indigo')
        plt.scatter(origin[0], origin[1], s=8, c='maroon')
        plt.xlabel(xlabels[iter])
        plt.ylabel(ylabels[iter])
        plt.xlim([-5, 10])
        plt.ylim([-10, 10])
        plt.title(titles[iter])
        # Plot physical space drawings of links 1 and 2
        for i in range(len(theta_1s)):
                link_1_angle = theta_1s[i]
                link_2_angle = theta_1s[i]+theta_2s[i]
                link_1_start = origin
                link_1_end=[link_1_start[0]+math.cos(link_1_angle)*length_1, 
                    link_1_start[1]+math.sin(link_1_angle)*length_1]
                link_2_start = link_1_end
                link_2_end=[link_2_start[0]+math.cos(link_2_angle)*length_2, 
                    link_2_start[1]+math.sin(link_2_angle)*length_2]
                plt.plot([link_1_start[0], link_1_end[0]], [link_1_start[1], link_1_end[1]], 'g--')
                plt.plot([link_2_start[0], link_2_end[0]], [link_2_start[1], link_2_end[1]], 'b--')
        iter = iter + 1

        # Plot Thetas vs X,Y plots
        theta_1s_deg = [math.degrees(theta) for theta in theta_1s]
        theta_2s_deg = [math.degrees(theta) for theta in theta_2s]
        for angles in [theta_1s_deg, theta_2s_deg]:
            axis = fig.add_subplot(subplots[iter])
            plt.scatter(relevant_xs, angles, s=8, c=['red'])
            plt.scatter(relevant_ys, angles, s=8, c=['blue'])
            plt.xlabel(xlabels[iter])
            plt.ylabel(ylabels[iter])
            plt.xlim([-10, 10])
            plt.ylim([-360, 360])
            plt.title(titles[iter])
            iter = iter + 1
    legend_elements = [Line2D([0], [0], linestyle='--', color='g', label=r'Link 1', lw=2),
                        Line2D([0], [0], linestyle='--', color='b', label=r'Link 2', lw=2),
                        Line2D([0], [0], marker='o', color='w', label=r'Origin',
                            markerfacecolor='maroon', markersize=8),
                        Line2D([0], [0], marker='o', color='w', label=r'TP',
                            markerfacecolor='indigo', markersize=8),
                        Line2D([0], [0], marker='o', color='w', label=r'X Values',
                            markerfacecolor='red', markersize=8),
                        Line2D([0], [0], marker='o', color='w', label=r'Y Values',
                            markerfacecolor='blue', markersize=8),]
    fig.legend(handles=legend_elements, loc='upper right')
    fig.tight_layout()
    
#Generates a plot on two levels:
#   Top Plots: x vs time, dx/dt vs time, dx^2/d^2 vs time
#   Bottom Plots: y vs time, dy/dt vs time, dy^2/d^2 vs time
# Specify angles_yes to label plots by (False) Cartesian (x,y) or (True) Angular (theta_1, theta_2)
def plot_profile_characteristics(time, positions, velocities, accelerations, angles_yes = False, figure_num=1):
    figure = plt.figure(figure_num)
    subplot_num = 1
    for y_axis_vals in [positions[0], velocities[0], accelerations[0],\
        positions[1], velocities[1], accelerations[1]]:
        
        figure.add_subplot(230+subplot_num)
        if subplot_num <=3:
            dim = r'$\theta_1$' if angles_yes else r'$x$'
        else:
            dim = r'$\theta_2$' if angles_yes else r'$y$'
        plt.ylabel(dim+' (deg)') if angles_yes else plt.ylabel(dim)
        plt.xlabel('time (sec)')
        if subplot_num % 3 == 2:
            plt.title(r'$d$'+dim+r'$/dt$'+' vs. ' r'$Time$')
        elif subplot_num % 3 == 0:
            plt.title(r'$d$'+dim+r'$^2/d^2t$'+' vs. '+r'$Time$')
        else:
            plt.title(dim+' vs. ' r'$Time$')
        plt.scatter(time, y_axis_vals, s=8, c='darkorange') if angles_yes \
            else plt.scatter(time, y_axis_vals, s=8, c='limegreen')

        subplot_num = subplot_num + 1; 
    figure.tight_layout()

def plot_torque_characteristics(start, end, time, sample_time, l1, l2, m1, m2, figure_start=1, max_torque=None):
    in_oz_per_newton_meter = 141.611933
    time_array, pos_xy, vel_xy, acc_xy = trap_profile_gen_test_points(start, end, time, sample_time)
    fig = plt.figure(figure_start)
    subplot_num = 1
    for elbow in [True, False]:
        theta_1s, theta_2s = two_link_jacobian_kinematics(time_array, pos_xy, vel_xy,\
            l1, l2, sample_time=sample_time, elbow_up=elbow)
        inches_per_meter = 1/(0.0254)
        l1_m = l1/inches_per_meter
        l2_m = l2/inches_per_meter
        [torques_theta_1, torques_theta_2] = torque_by_lagrangian(theta_1s, theta_2s, m1, m2, \
            l1_m, l2_m, time_array)
        for torques in [torques_theta_1, torques_theta_2]:
            fig.add_subplot(220+subplot_num)
            #Make sure we plot in oz-in
            torque_total = [torques[0][k]*in_oz_per_newton_meter for k in range(len(torques[0]))]
            torque_inertial = [torques[1][k]*in_oz_per_newton_meter for k in range(len(torques[1]))]
            torque_coriolis = [torques[2][k]*in_oz_per_newton_meter for k in range(len(torques[2]))]
            torque_centripetal = [torques[3][k]*in_oz_per_newton_meter for k in range(len(torques[3]))]
            plt.plot(time_array, torque_total, 'orange', label=r'T_(total)', linewidth=3)
            plt.plot(time_array, torque_inertial, 'r--', label=r'T_(inertial)')
            plt.plot(time_array, torque_coriolis, 'g--', label=r'T_(coriolis)')
            plt.plot(time_array, torque_centripetal, 'b--', label=r'T_(centripetal)')
            if elbow:
                if subplot_num%2 == 1:
                    plt.title(r'$\theta_1$'+r' Torques vs. Time: Elbow Up')
                    plt.ylabel(r'$T_{\theta_1}$'+r'(oz-in)')
                else:
                    plt.title(r'$\theta_2$'+r' Torques vs. Time: Elbow Up')
                    plt.ylabel(r'$T_{\theta_2}$'+r'(oz-in)')
            else:
                if subplot_num%2 == 1:
                    plt.title(r'$\theta_1$'+r' Torques vs. Time: Elbow Down')
                    plt.ylabel(r'$T_{\theta_1}$'+r'(oz-in)')
                else:
                    plt.title(r'$\theta_2$'+r' Torques vs. Time: Elbow Down')
                    plt.ylabel(r'$T_{\theta_2}$'+r'(oz-in)')
            if not max_torque == None:
                #Assume max_torque given as in-oz
                plt.hlines(max_torque, time_array[0], time_array[-1], colors='m', linewidth=1)
                plt.hlines(-1*max_torque, time_array[0], time_array[-1], colors='m', linewidth=1)
            plt.xlabel("Time (sec)")
            subplot_num = subplot_num+1
        if not max_torque == None:
            legend_elements = [Line2D([0], [0], linestyle='-', color='orange', label=r'$T_{total}$', lw=3),
                            Line2D([0], [0], linestyle='--', color='r', label=r'$T_{inertial}$', lw=2),
                            Line2D([0], [0], linestyle='--', color='g', label=r'$T_{coriolas}$', lw=2),
                            Line2D([0], [0], linestyle='--', color='b', label=r'$T_{centripetal}$', lw=2),
                            Line2D([0], [0], linestyle='-', color='m', label=r'$T_{stall}$', lw=1)]
        else:
            legend_elements = [Line2D([0], [0], linestyle='-', color='orange', label=r'$T_{total}$', lw=3),
                            Line2D([0], [0], linestyle='--', color='r', label=r'$T_{inertial}$', lw=2),
                            Line2D([0], [0], linestyle='--', color='g', label=r'$T_{coriolas}$', lw=2),
                            Line2D([0], [0], linestyle='--', color='b', label=r'$T_{centripetal}$', lw=2)]
        fig.legend(handles=legend_elements, loc='upper left')
        fig.tight_layout()
    pass

#Generates a trapezoidal velocity profile
#  Acceleration time : ta
#  Constant velocity region time : tcv
#  Decceleration time : td; unused; is set equal to ta
#  Displacement: displacement;  in units specified
#  Sample Time: sample_time; how long it takes to complete path
#Returns:
#   Time Points array, positions array, velocity array, acceleration array
def trap_profile_gen(ta, tcv, td, displacement, sample_time): 
    #Adapted from Matlab (Credit: T. Chmielewski)
    #Time to nearest increment
    time_accel = round(ta/sample_time)*sample_time
    # time_deccel = round(td/sample_time)*sample_time
    time_deccel = time_accel #Force equal accel/deccel times
    time_const_vel = round(tcv/sample_time)*sample_time

    total_time = time_accel+time_const_vel+time_deccel
    vel_max = displacement/(time_accel+time_const_vel)
    accel_max = vel_max/time_accel
    deccel_max = accel_max
    computed_displacement = (0.5*(time_accel+time_deccel)+time_const_vel)*vel_max

    num_updates = int(total_time/sample_time)

    times = []
    accels = []
    velocities = []
    positions = []

    a_prev = 0
    v_prev = 0
    x_prev = 0
    x_now = 0

    xtra_count_num = 0
    #Determine the time vector
    for time in range(num_updates+xtra_count_num):
        t_now = time*sample_time
        times.append(time)
        #Acceleration
        if (t_now < time_accel) and (t_now >= 0):
            a_now = accel_max
        elif (t_now >= (time_accel+time_const_vel)) and (t_now >= 0):
            a_now = -1*deccel_max
        else:
            a_now = 0

        #Velocity
        v_now = v_prev + sample_time*a_now
        x_now = x_prev + (sample_time/2)*(v_now+v_prev) #Trapezoidal
        accels.append(a_now)
        velocities.append(v_now)
        positions.append(x_now)
        v_prev = v_now
        x_prev = x_now
    
    position_error = computed_displacement - x_now
    if (abs(position_error) > 0):
        print("Position Error: "+str(position_error))
    return [times, positions, velocities, accels]

#Generates a trapezoidal velocity profile like trap_profile_gen with 1/3-1/3-1/3 times
def trap_profile_gen_ez(total_time, displacement, sample_time):
    return trap_profile_gen(total_time/3.0, total_time/3.0, total_time/3.0,\
        displacement, sample_time)

#Generates position, velocity, and acceleration arrays with respect to each degree of freedom (2; x, y)
#Returns same as trap_profile_gen, but position, velocity, and acceleration has internal arrays for x and y
def trap_profile_gen_test_points(start_point, end_point, total_time, sample_time):
    displacement = math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
    time_array, x_array, v_array, a_array = trap_profile_gen_ez(total_time, displacement, sample_time)

    position_profile = [[], []]
    velocity_profile = [[], []]
    acceleration_profile = [[], []]
    angle_from_start = math.atan((end_point[1]-start_point[1])/(end_point[0]-start_point[0]))
    for position in x_array:
        position_profile[0].append(start_point[0]+position*math.cos(angle_from_start))
        position_profile[1].append(start_point[1]+position*math.sin(angle_from_start))
    for velocity in v_array:
        velocity_profile[0].append(velocity*math.cos(angle_from_start))
        velocity_profile[1].append(velocity*math.sin(angle_from_start))
    for acceleration in a_array:
        acceleration_profile[0].append(acceleration*math.cos(angle_from_start))
        acceleration_profile[1].append(acceleration*math.sin(angle_from_start))

    return [time_array, position_profile, velocity_profile, acceleration_profile]

#Calculates the torques experienced at each joint points
#Returns:
#   A 3D array. Each row corresponds to theta_1 and then theta_2
#   Each row contains 4 sub arrays
#       [[[total_torque_1], [interial_torque_1], [coriolis_torque_1], [centripetal_torque_1]];
#       [[total_torque_2], [interial_torque_2], [coriolis_torque_2], [centripetal_torque_2]]
def torque_by_lagrangian(theta_1s, theta_2s, mass_1, mass_2, length_1, length_2, time_array):
    theta_dot_array = position_to_velocity(time_array, theta_1s, theta_2s)
    theta_ddot_array = position_to_acceleration(time_array, theta_1s, theta_2s)
    torque_total_1 = []
    torque_inertial_1 = []
    torque_coriolis_1 = []
    torque_centripetal_1 = []
    torque_total_2 = []
    torque_inertial_2 = []
    torque_coriolis_2 = []
    torque_centripetal_2 = []
    for i in range(2):
        for j in range(len(time_array)):
            [d1, d2, m1, m2] = [length_1, length_2, mass_1, mass_2]
            [theta_1, theta_2] = [theta_1s[j], theta_2s[j]]
            [theta_dot_1, theta_dot_2] = [theta_dot_array[0][j], theta_dot_array[1][j]]
            [theta_ddot_1, theta_ddot_2] = [theta_ddot_array[0][j], theta_ddot_array[1][j]]
            if i == 0:
                inertial = theta_ddot_1*(
                    (d1**2)*(m1+m2)+
                    m2*(d2**2)+
                    2*m2*d1*d2*math.cos(theta_2)
                )
                coupled_inertial = theta_ddot_2*(
                    m2*(d2**2)+
                    m2*d1*d2*math.cos(theta_2)
                )
                coriolis = theta_dot_1*theta_dot_2*(
                    2*m2*d1*d2*math.sin(theta_2)
                )
                centripetal = (theta_dot_2**2)*(
                    m2*d1*d2*math.sin(theta_2)
                )
                torque_inertial_1.append(inertial+coupled_inertial)
                torque_coriolis_1.append(-1*coriolis)
                torque_centripetal_1.append(-1*centripetal)
                torque_total_1.append(inertial+coupled_inertial-coriolis-centripetal)
            elif i == 1:
                inertial = theta_ddot_1*(
                    m2*(d2**2)+
                    m2*d1*d2*math.cos(theta_2)
                )
                coupled_inertial = theta_ddot_2*(
                    m2*(d2**2)
                )
                coriolis = 0
                centripetal = (theta_dot_1**2)*(
                    m2*d1*d2*math.sin(theta_2)
                )
                torque_inertial_2.append(inertial+coupled_inertial)
                torque_coriolis_2.append(-1*coriolis)
                torque_centripetal_2.append(-1*centripetal)
                torque_total_2.append(inertial+coupled_inertial-coriolis-centripetal)
            else:
                pass
    return [[torque_total_1, torque_inertial_1, torque_coriolis_1, torque_centripetal_1], 
            [torque_total_2, torque_inertial_2, torque_coriolis_2, torque_centripetal_2]]

if __name__ == "__main__":
    #Configure system characteristics
    start = [2, 8]
    end = [9, 2]
    manipulator_base_origin = [0, 0]
    link_lengths = 5 #inches
    m1 = 1 #kg
    m2 = 4*m1 #kg
    stall_torque_top = 56.93 #oz-in
    stall_torque_bottom = 45.82 #oz-in

    # Find points on line according to velocity profile
    time = 1.8 #Seconds (Fixed for all functions)
    sample_time = 0.05 #50 milliseconds (Fixed for all functions)

    plot_torque_characteristics(start, end, time, sample_time, link_lengths, link_lengths, m1, m2, figure_start=1)
    plt.show()


    
