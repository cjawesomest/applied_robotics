# Cameron J. Calv 
# ECES T580 Applied Robotics
# Programming Assignment 2
#   Velocity profiles along a straight line (and position and acceleration too)
import math
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

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
def plot_kinematic_figures(tp_xs, tp_ys, length_1, length_2, origin, figure_num=1):
    #Set up plot values
    fig = plt.figure(figure_num)
    subplots = [242, 245, 246, 243, 247, 248]
    titles = ['Elbow Up : Space', r'Elbow Up : X, Y and $\theta_1$', r'Elbow Up : X, Y and $\theta_2$',
        'Elbow Down : Space', r'Elbow Down : X, Y and $\theta_1$', r'Elbow Down : X, Y and $\theta_2$']
    xlabels = ['x', 'x,y', 'x,y', 'x', 'x,y', 'x,y']
    ylabels = ['y', r'$\theta_1$ (deg)', r'$\theta_2$ (deg)', 'y', r'$\theta_1$ (deg)', r'$\theta_2$ (deg)']
    iter = 0
    for elbow in [True, False]:
        theta_1s, theta_2s = two_link_inverse_kinematics(tp_xs, tp_ys, length_1, length_2, elbow_up=elbow)
        
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
            plt.ylim([-180, 180])
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

if __name__ == "__main__":
    #Linear Movement
    x_values_linear, y_values_linear = linear_gen(-2, 10, 6, 1, 7)
    start = [x_values_linear[0], y_values_linear[0]]
    end = [x_values_linear[-1], y_values_linear[-1]]

    manipulator_base_origin = [0, 0]
    link_lengths = 5

    # Find points on line according to velocity profile
    time = 1.8 #Seconds
    sample_time = 0.05 #50 milliseconds

    # Determine profiles
    time_array, pos_xy, vel_xy, acc_xy = trap_profile_gen_test_points(start, end, time, sample_time)

    # Inverse kinematics for each test position
    theta_1s, theta_2s = two_link_inverse_kinematics(pos_xy[0],\
        pos_xy[1], link_lengths, link_lengths, elbow_up=True)

    # Plot manipulator and thetas vs positions
    plot_kinematic_figures(pos_xy[0], pos_xy[1], 
        link_lengths, link_lengths, manipulator_base_origin, figure_num = 1)

    # Plot x,y positions, velocities, and accelerations vs time
    plot_profile_characteristics(time_array, pos_xy, vel_xy, acc_xy, angles_yes = False, figure_num = 2)

    theta_1s_deg = [math.degrees(theta) for theta in theta_1s]
    theta_2s_deg = [math.degrees(theta) for theta in theta_2s]

    v_theta = position_to_velocity(time_array, theta_1s_deg, theta_2s_deg)
    a_theta = position_to_acceleration(time_array, theta_1s_deg, theta_2s_deg)
    # Plot theta1, theta2 positions, velocities, and accelerations vs time
    plot_profile_characteristics(time_array, [theta_1s_deg, theta_2s_deg], v_theta, a_theta, angles_yes = True, figure_num = 3)
    
    plt.show()
        


    
