# Cameron J. Calv 
# ECES T580 Applied Robotics
# Programming Assignment 1
#   Inverse Kinematics on arbitrary quadrant I & IV lines
import math
import matplotlib

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

#Generates 2 figures: 
#   Figure 1: Showing RR manipulator in physical space with 2 links extending to test pionts
#   Figure 2: Creating plots showcasing theta_1/2 vs x/y for elbow up and down positions
def plot_kinematic_figures(tp_xs, tp_ys, length_1, length_2, origin, figure_num=1):
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

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
    
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    #Elliptical Movement
    x_values_ellipse, y_values_ellipse = ellipse_gen(r=8, x_step=0.5, x_final=10)
    #Linear Movement
    x_values_linear, y_values_linear = linear_gen(-2, 10, 1, 0, 10)

    manipulator_base_origin = [0, 0]
    link_lengths = 5


    plot_kinematic_figures(x_values_ellipse, y_values_ellipse, 
        link_lengths, link_lengths, manipulator_base_origin, figure_num = 1)
    plot_kinematic_figures(x_values_linear, y_values_linear, 
        link_lengths, link_lengths, manipulator_base_origin, figure_num = 2)
        
    plt.show()