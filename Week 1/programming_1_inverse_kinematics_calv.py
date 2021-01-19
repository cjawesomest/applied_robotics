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

#Takes an array of radian values and converts

#Determines the link angles of a two-link manipulator for an array of end-effector positions
def two_link_inverse_kinematics(x_vals, y_vals, length_1, length_2, elbow_up=True):
    theta_1s = []
    theta_2s = []
    if not len(x_vals)==len(y_vals):
        return theta_1s, theta_2s
    if elbow_up:
        sigma = 1
    else:
        sigma = -1
    for i in range(len(x_vals)):
        this_x = x_vals[i]
        this_y = y_vals[i]
        med_root = math.sqrt((this_x**2)+(this_y**2))
        gamma = math.atan2((-1)*this_y/med_root,this_x/med_root)
        # gamma = math.atan(((this_x/med_root)-(this_y/med_root)))**2
        med_cos = ((this_x**2)+(this_y**2)+(length_1**2)-(length_2**2))/(2*length_1*med_root)
        if med_cos > 1:
            med_cos = 1
        elif med_cos < -1:
            med_cos = -1
        this_theta_1 = gamma + sigma*math.acos(med_cos)
        med_cos = (this_x - length_1*math.cos(this_theta_1))/length_2
        med_sin = (this_y - length_1*math.sin(this_theta_1))/length_2
        if med_cos > 1:
            med_cos = 1
        elif med_cos < -1:
            med_cos = -1
        if med_sin > 1:
            med_sin = 1
        elif med_sin < -1:
            med_sin = -1
        #Attempting various solutions for theta 2
        if this_theta_1 >= 0:
            this_theta_2 = math.acos(med_cos)-this_theta_1
        else:
            this_theta_2 = math.acos(med_cos)+this_theta_1
        # this_theta_2 = math.asin(med_sin)-this_theta_1
        # this_theta_2 = math.atan2(med_cos, med_sin)-this_theta_1
        theta_1s.append(this_theta_1)
        theta_2s.append(this_theta_2)
    return theta_1s, theta_2s


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    #Elliptical Movement
    x_values_ellipse, y_values_ellipse = ellipse_gen(r=10, x_step=0.5, x_final=10)
    x_values_linear, y_values_linear = linear_gen(-1, 10, 1, 0, 10)

    print(max(x_values_ellipse))

    link_lengths = 5
    manipulator_base_origin = [0, 0]

    #Set up plot details
    fig = plt.figure() 
    subplot=220
    count = 0
    for elbow in [True, False, True, False]:
        subplot = subplot+1
        axis = fig.add_subplot(subplot)

        if count < 2:
            plt.scatter(x_values_ellipse, y_values_ellipse)
            theta_1s, theta_2s = two_link_inverse_kinematics(x_values_ellipse, y_values_ellipse, link_lengths, link_lengths, elbow_up=elbow)
            if elbow:
                plt.title("Elbow Up Elliptical")
            else:
                plt.title("Elbow Down Elliptical")
        else:
            plt.scatter(x_values_linear, y_values_linear)
            theta_1s, theta_2s = two_link_inverse_kinematics(x_values_linear, y_values_linear, link_lengths, link_lengths, elbow_up=elbow)
            if elbow:
                plt.title("Elbow Up Linear")
            else:
                plt.title("Elbow Down Linear")
        
        axis.set_aspect('equal', adjustable='box') #For a more circular looking circle
        # axis.set_aspect(1./axis.get_data_ratio()) #For a wider looking circle
        plt.xlim([-1, 2*link_lengths])
        plt.ylim([-2*link_lengths, 2*link_lengths])

        #Plot links extending to each point on plot
        for i in range(len(theta_1s)):
            link_1_angle = theta_1s[i]
            link_2_angle = theta_1s[i]+theta_2s[i]
            link_1_start = manipulator_base_origin
            link_1_end=[link_1_start[0]+math.cos(link_1_angle)*link_lengths, 
                link_1_start[1]+math.sin(link_1_angle)*link_lengths]
            link_2_start = link_1_end
            link_2_end=[link_2_start[0]+math.cos(link_2_angle)*link_lengths, 
                link_2_start[1]+math.sin(link_2_angle)*link_lengths]
            plt.plot([link_1_start[0], link_1_end[0]], [link_1_start[1], link_1_end[1]], 'g--')
            plt.plot([link_2_start[0], link_2_end[0]], [link_2_start[1], link_2_end[1]], 'b--')
        count = count + 1
    plt.show()
    theta_1s_deg = [math.degrees(theta) for theta in theta_1s]
    theta_2s_deg = [math.degrees(theta) for theta in theta_2s]
    print(theta_1s_deg)
    print(theta_2s_deg)