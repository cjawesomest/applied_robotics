# Cameron J. Calv 
# ECES T580 Applied Robotics
# Programming Assignment 1
#   Inverse Kinematics on arbitrary quadrant I & IV lines
import math

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
    this_x = x_init
    for i in range(math.ceil((x_final-x_init)/x_step)+1):
        med = (a**2)*(b**2)*((a**2)*(r**2)-(center_x**2)+2*center_x*this_x-(this_x**2))
        if med < 0:
            continue
        xs.append(this_x)
        ys.append(((a**2)*center_y-math.sqrt(med))/(a**2))
        xs.append(this_x)
        ys.append(((a**2)*center_y+math.sqrt(med))/(a**2))
        this_x = this_x + x_step
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
        gamma = math.atan(2*((this_x/med_root)-(this_y/med_root)))
        med_cos = ((this_x**2)+(this_y**2)+(length_1**2)-(length_2**2))/(2*length_1*med_root)
        if med_cos > 1:
            med_cos = 1
        elif med_cos < -1:
            med_cos = -1
        this_theta_1 = gamma + sigma*math.acos(med_cos)
        med_cos = (this_x - length_1*math.cos(this_theta_1))/length_2
        if med_cos > 1:
            med_cos = 1
        elif med_cos < -1:
            med_cos = -1
        this_theta_2 = math.acos(med_cos)-this_theta_1
        theta_1s.append(this_theta_1)
        theta_2s.append(this_theta_2)
    return theta_1s, theta_2s


if __name__ == "__main__":
    x_values, y_values = ellipse_gen()
    theta_1s, theta_2s = two_link_inverse_kinematics(x_values, y_values, 0.5, 0.5, elbow_up=True)
    theta_1s_deg = [math.degrees(theta) for theta in theta_1s]
    theta_2s_deg = [math.degrees(theta) for theta in theta_2s]
    print(theta_1s_deg)
    print(theta_2s_deg)