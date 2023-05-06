import numpy as np

KP_POS = 3
KP_NEG = 10
OBST_RADIUS = 1

def gradient_attraction(pos, goal):
    d_x = pos[0] - goal[0]
    d_y = pos[1] - goal[1]
    d_star = 0.5

    dist = np.sqrt(d_x**2 + d_y**2)
    
    if dist <= d_star:
        x = 1 * KP_POS * d_x
        y = 1 * KP_POS * d_y
    else:
        x = 1 * KP_POS * d_star * d_x / dist
        y = 1 * KP_POS * d_star * d_y / dist

    return [x,y]

def gradient_repulsion(pos, obstacle):
    x_p = pos[0]
    x_o = obstacle[0]
    y_p = pos[1]
    y_o = obstacle[1]

    d_x = x_p - x_o
    d_y = y_p - y_o

    dist = np.sqrt(d_x**2 + d_y**2)
    if dist > OBST_RADIUS:
        return [0,0]

    x = KP_NEG * ((1/OBST_RADIUS) - 1/(dist)) * (1/((dist)**2)) * d_x
    y = KP_NEG * ((1/OBST_RADIUS) - 1/(dist)) * (1/((dist)**2)) * d_y
    #x = KP_NEG * (2 * (x_p/1 - x_g/1))/(((x_g/30 - x_p/30)**2 + (y_g/30 - y_p/30)**2)**2)
    #y = KP_NEG * (2 * (y_p/1 - y_g/1))/(((x_g/30 - x_p/30)**2 + (y_g/30 - y_p/30)**2)**2)
    return [x,y]

def gradient(pos, goal, obstacles):

    pos = np.array(pos)
    goal = np.array(goal)
    obstacles = np.array(obstacles)

    gradient_arr = []
    gradient_arr.append(gradient_attraction(pos, goal))
    
    for r in obstacles:
        gradient_arr.append(gradient_repulsion(pos, r))

    x = 0.0
    y = 0.0
    for grad in gradient_arr:
        x += grad[0]
        y += grad[1]
    
    x = x
    y = y

    return [x,y]