import numpy as np

KP_POS = 1
KP_NEG = 1
OBST_RADIUS = 1

def gradient_attraction(pos, goal):
    d_x = pos[0] - goal[0]
    d_y = pos[1] - goal[1]
    d_star = 0.2

    dist = np.sqrt(d_x**2 + d_y**2)

    if dist <= d_star:
        x = 1 * KP_POS * d_x
        y = 1 * KP_POS * d_y

    else:
        x = KP_POS * d_star * d_x / dist
        y = KP_POS * d_star * d_y / dist

    return [x,y]

def gradient_repulsion(pos, obstacle):
    x_p = pos[0]
    x_g = obstacle[0]
    y_p = pos[1]
    y_g = obstacle[1]

    d_x = x_p - x_g
    d_y = y_p - y_g
    if d_x < 1:
        d_x = 1
    if d_y < 1:
        d_y = 1

    x = KP_NEG * ((1/OBST_RADIUS) - 1/(d_x)) * (1/((d_x)**2))
    y = KP_NEG * ((1/OBST_RADIUS) - 1/(d_y)) * (1/((d_y)**2))
    #x = KP_NEG * (2 * (x_p/1 - x_g/1))/(((x_g/30 - x_p/30)**2 + (y_g/30 - y_p/30)**2)**2)
    #y = KP_NEG * (2 * (y_p/1 - y_g/1))/(((x_g/30 - x_p/30)**2 + (y_g/30 - y_p/30)**2)**2)
    return [x,y]

def gradient(pos, goal, obstacles):

    pos = np.array(pos) * 10
    goal = np.array(goal) * 10
    obstacles = np.array(obstacles) * 10

    gradient_arr = []
    gradient_arr.append(gradient_attraction(pos, goal))
    
    for r in obstacles:
        gradient_arr.append(gradient_repulsion(pos, r))

    x = 0.0
    y = 0.0
    for grad in gradient_arr:
        x += grad[0]
        y += grad[1]
    
    x = x/10
    y = y/10

    return [x,y]