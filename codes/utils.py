import os
import numpy as np
import gym
import gym_minigrid
import pickle
import matplotlib.pyplot as plt
import imageio
import random

MF = 0 # Move Forward
TL = 1 # Turn Left
TR = 2 # Turn Right
PK = 3 # Pickup Key
UD = 4 # Unlock Door

move = [np.array([0,1,0]),np.array([-1,0,0]),np.array([0,-1,0]),np.array([1,0,0])]

def openup(x):
    return x[0],x[1],x[2]

def d2tod3(s):
    return np.array([s[0], s[1], 0])

def dir_change(arg,pos):
    #assert arg == TL or arg == TR
    ans = pos[2]
    if arg == TL:
        ans -= 1
        if ans == -1:
            ans = 3
    else:
        ans += 1
        if ans == 4:
            ans = 0
    pos[2] = ans
    return pos

'''
Direction convention:
  2
1-|-3
  0
'''

# Tells if the given coordinates are in boundry or not
def inbound(pos, envn):
    x = envn.width - 1
    y = envn.height - 1
    if 0<=pos[0]<=x and 0<=pos[1]<=y:
        return True
    else:
        return False

# whether the path is blocked by closed door, door_status = islocked
def door_block(pos, door_pos):
    p = np.array(pos[0], pos[1])
    d = np.array(door_pos)

    if np.array_equal(p, d):
        return True

def door_block3(pos, door_pos, door_status):
    p = np.array(pos[0], pos[1])
    d = np.array(door_pos)

    if np.array_equal(p, d) and door_status:
        return True
    else:
        return False
# Gives possible children of nodes w.r.t. environment
def children(start, envn, door_pos, locked = True):
    chld = []
    s = np.array(start)
    
    fw = s+move[s[2]]
    if envn.grid.get(fw[0],fw[1]) is None:
        chld.append(fw)
    elif envn.grid.get(fw[0],fw[1]).type != 'wall':
        if door_block(fw, door_pos) is False:
            chld.append(fw)
        elif locked == False:
            chld.append(fw)
        
    a = dir_change(TL,s.copy())
    chld.append(a)
    b = dir_change(TR,s.copy())
    chld.append(b)
    
    return chld

def children2(start, envn, door_pos, door_pos2, locked = True):
    if envn.grid.get(door_pos[0], door_pos[1]).type == 'wall':
        door_status1 = False
    else:
        door_status1 = envn.grid.get(door_pos[0], door_pos[1]).is_locked
    if envn.grid.get(door_pos2[0], door_pos2[1]).type == 'wall':
        door_status2 = False
    else:
        door_status2 = envn.grid.get(door_pos2[0], door_pos2[1]).is_locked
    chld = []
    s = np.array(start)
    
    fw = s+move[s[2]]
    if envn.grid.get(fw[0],fw[1]) is None:
        chld.append(fw)
    elif envn.grid.get(fw[0],fw[1]).type != 'wall':
        if door_block(fw, door_pos) is False and door_block(fw, door_pos):
            chld.append(fw)
        elif locked == False:
            chld.append(fw)
        
    a = dir_change(TL,s.copy())
    chld.append(a)
    b = dir_change(TR,s.copy())
    chld.append(b)
    
    return chld

def children3(start, envn, door_pos, locked = True):
    if envn.grid.get(door_pos[0], door_pos[1]).type == 'wall':
        door_status1 = False
    else:
        door_status1 = envn.grid.get(door_pos[0], door_pos[1]).is_locked
    chld = []
    s = np.array(start)
    
    fw = s+move[s[2]]
    if envn.grid.get(fw[0],fw[1]) is None:
        chld.append(fw)
    elif envn.grid.get(fw[0],fw[1]).type != 'wall':
        if door_block3(fw, door_pos, door_status1) is False:
            chld.append(fw)
        elif locked == False:
            chld.append(fw)
        
    a = dir_change(TL,s.copy())
    chld.append(a)
    b = dir_change(TR,s.copy())
    chld.append(b)
    
    return chld

# Mapping state space to control inputs
def control(path):
    ans = []
    for i in range(len(path)-1):
        nxt = path[i+1]
        curr = path[i]
        if nxt[0] - curr[0] == 0 and nxt[1] - curr[1] == 0:
            dr = nxt[2] - curr[2]
            if dr == -1 or dr == 3:
                u = TL
            else:
                u = TR
        else:
            u = MF
        ans.append(u)
    return ans

# Label correcting algorythms
def dp(start, end, envn, door_pos, locked = True):
    parent = {}
    queue = [start]
    if np.array_equal(end, start):
        return [], start
    V = np.ones((envn.height,envn.width,4))*np.inf
    V[tuple(start)] = 0

    while queue:
        i = queue.pop()
        childs = children(i, envn, door_pos, locked)
        for j in childs:
            if V[openup(i)]+1 < V[openup(j)]:
                V[openup(j)] = V[openup(i)]+1
                parent[tuple(j)] = i
                if np.array_equal(j, end) == False:
                    queue.append(j)

    # if there doesn't exist an open path from start to end               
    if tuple(end) not in parent:
        return np.ones((envn.height*envn.width*4))*MF, start
    # If there exists an open path
    r_from = end
    ulta_path = []
    i = 2
    while i>1:
        node = parent[tuple(r_from)]
        ulta_path.append(node)
        if np.array_equal(node, start):
            break
        else:
            r_from = node
    
    path = ulta_path[::-1]
    path.append(end)

    policy = control(path)

    return policy, path[-1]

def dp2(start, end, envn, door_pos1, door_pos2, locked = True):
    parent = {}
    queue = [start]
    if np.array_equal(end, start):
        return [], start
    V = np.ones((envn.height,envn.width,4))*np.inf
    V[tuple(start)] = 0

    while queue:
        i = queue.pop()
        childs = children2(i, envn, door_pos1, door_pos2, locked)
        for j in childs:
            if V[openup(i)]+1 < V[openup(j)]:
                V[openup(j)] = V[openup(i)]+1
                parent[tuple(j)] = i
                if np.array_equal(j, end) == False:
                    queue.append(j)

    # if there doesn't exist an open path from start to end               
    if tuple(end) not in parent:
        return np.ones((envn.height*envn.width*4))*MF, start
    # If there exists an open path
    r_from = end
    ulta_path = []
    i = 2
    while i>1:
        node = parent[tuple(r_from)]
        ulta_path.append(node)
        if np.array_equal(node, start):
            break
        else:
            r_from = node
    
    path = ulta_path[::-1]
    path.append(end)

    policy = control(path)

    return policy, path[-1]
    
def dp3(start, end, envn, locked = True):
    #door_status = envn.grid.get(door_pos[0], door_pos[1]).is_locked
    parent = {}
    queue = [start]
    if np.array_equal(end, start):
        return [], start
    V = np.ones((envn.height,envn.width,4))*np.inf
    V[tuple(start)] = 0

    while queue:
        i = queue.pop()
        childs = children3(i, envn, [0,0,0], locked)
        for j in childs:
            if V[openup(i)]+1 < V[openup(j)]:
                V[openup(j)] = V[openup(i)]+1
                parent[tuple(j)] = i
                if np.array_equal(j, end) == False:
                    queue.append(j)

    # if there doesn't exist an open path from start to end               
    if tuple(end) not in parent:
        return np.ones((envn.height*envn.width*4))*MF, start
    # If there exists an open path
    r_from = end
    ulta_path = []
    i = 2
    while i>1:
        node = parent[tuple(r_from)]
        ulta_path.append(node)
        if np.array_equal(node, start):
            break
        else:
            r_from = node
    
    path = ulta_path[::-1]
    path.append(end)

    policy = control(path)

    return policy, path[-1] 


def step_cost(action):
    # You should implement the stage cost by yourself
    # Feel free to use it or not
    # ************************************************
    if action == 0:
        return 1
    elif action == 1:
        return 1
    elif action == 2:
        return 1
    elif action == 3:
        return 1
    elif action == 4:
        return 1

def step(env, action):
    '''
    Take Action
    ----------------------------------
    actions:
        0 # Move forward (MF)
        1 # Turn left (TL)
        2 # Turn right (TR)
        3 # Pickup the key (PK)
        4 # Unlock the door (UD)
    '''
    actions = {
        0: env.actions.forward,
        1: env.actions.left,
        2: env.actions.right,
        3: env.actions.pickup,
        4: env.actions.toggle
        }

    _, _, done, _ = env.step(actions[action])
    return step_cost(action), done

def generate_random_env(seed, task):
    ''' 
    Generate a random environment for testing
    -----------------------------------------
    seed:
        A Positive Integer,
        the same seed always produces the same environment
    task:
        'MiniGrid-DoorKey-5x5-v0'
        'MiniGrid-DoorKey-6x6-v0'
        'MiniGrid-DoorKey-8x8-v0'
    '''
    if seed < 0:
        seed = np.random.randint(50)
    env = gym.make(task)
    env.seed(seed)
    env.reset()
    return env

def load_env(path):
    '''
    Load Environments
    ---------------------------------------------
    Returns:
        gym-environment, info
    '''
    with open(path, 'rb') as f:
        env = pickle.load(f)
    
    info = {
        'height': env.height,
        'width': env.width,
        'init_agent_pos': env.agent_pos,
        'init_agent_dir': env.dir_vec
        }
    
    for i in range(env.height):
        for j in range(env.width):
            if isinstance(env.grid.get(j, i),
                          gym_minigrid.minigrid.Key):
                info['key_pos'] = np.array([j, i])
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Door):
                info['door_pos'] = np.array([j, i])
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Goal):
                info['goal_pos'] = np.array([j, i])    
            
    return env, info

def load_random_env(env_folder):
    '''
    Load a random DoorKey environment
    ---------------------------------------------
    Returns:
        gym-environment, info
    '''
    env_list = [os.path.join(env_folder, env_file) for env_file in os.listdir(env_folder)]
    env_path = random.choice(env_list)
    with open(env_path, 'rb') as f:
        env = pickle.load(f)
    
    info = {
        'height': env.height,
        'width': env.width,
        'init_agent_pos': env.agent_pos,
        'init_agent_dir': env.dir_vec,
        'door_pos': [],
        'door_open': [],
        }
    
    for i in range(env.height):
        for j in range(env.width):
            if isinstance(env.grid.get(j, i),
                          gym_minigrid.minigrid.Key):
                info['key_pos'] = np.array([j, i])
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Door):
                info['door_pos'].append(np.array([j, i]))
                if env.grid.get(j, i).is_open:
                    info['door_open'].append(True)
                else:
                    info['door_open'].append(False)
            elif isinstance(env.grid.get(j, i),
                            gym_minigrid.minigrid.Goal):
                info['goal_pos'] = np.array([j, i])    
            
    return env, info, env_path

def save_env(env, path):
    with open(path, 'wb') as f:
        pickle.dump(env, f)

def plot_env(env):
    '''
    Plot current environment
    ----------------------------------
    '''
    img = env.render('rgb_array', tile_size=32)
    plt.figure()
    plt.imshow(img)
    plt.show()
    
def draw_gif_from_seq(seq, env, path):
    '''
    Save gif with a given action sequence
    ----------------------------------------
    seq:
        Action sequence, e.g [0,0,0,0] or [MF, MF, MF, MF]
    
    env:
        The doorkey environment
    '''
    with imageio.get_writer(path, mode='I', duration=0.8) as writer:
        img = env.render('rgb_array', tile_size=32)
        writer.append_data(img)
        for act in seq:
            img = env.render('rgb_array', tile_size=32)
            step(env, act)
            writer.append_data(img)
    print('GIF is written to {}'.format(path))
    return
 