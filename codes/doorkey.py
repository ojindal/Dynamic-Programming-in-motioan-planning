import numpy as np
import gym
from utils import *

MF = 0 # Move Forward
TL = 1 # Turn Left
TR = 2 # Turn Right
PK = 3 # Pickup Key
UD = 4 # Unlock Door

'''
Direction convention:
  2
1-|-3
  0
'''
# Conversion of direction from given form to my form
dirr = {tuple(np.array([0, 1])):0,
        tuple(np.array([-1, 0])):1,
        tuple(np.array([0, -1])):2,
        tuple(np.array([1, 0])):3}

# Movement cells
move1 = [np.array([-1,0,0]),np.array([0,-1,0]),np.array([1,0,0]),np.array([0,1,0])]
dir1 = [3, 0, 1, 2]
# Gives possible approach of nodes w.r.t. environment

# Cells from which a block can be approached
def approach(start, envn, door_pos):
    chld = []
    s = np.array(start)
    for i in range(len(move1)):
        cell = s+move1[i]
        if envn.grid.get(cell[0],cell[1]) is None:
            cell[2] = dir1[i]
            chld.append(cell)
        elif envn.grid.get(cell[0],cell[1]).type != 'wall':
            if door_block(cell, door_pos) is False:
                cell[2] = dir1[i]
                chld.append(cell)
    return chld

def approach2(start, envn, door_pos1,door_pos2):
    chld = []

    if envn.grid.get(door_pos1[0], door_pos1[1]).type == 'wall':
        door_status1 = False
    else:
        door_status1 = envn.grid.get(door_pos1[0], door_pos1[1]).is_locked
    if envn.grid.get(door_pos2[0], door_pos2[1]).type == 'wall':
        door_status2 = False
    else:
        door_status2 = envn.grid.get(door_pos2[0], door_pos2[1]).is_locked

    s = np.array(start)
    for i in range(len(move1)):
        cell = s+move1[i]
        if envn.grid.get(cell[0],cell[1]) is None:
            cell[2] = dir1[i]
            chld.append(cell)
        elif envn.grid.get(cell[0],cell[1]).type != 'wall':
            if door_block(cell, door_pos1, door_status1) is False and door_block(cell, door_pos2, door_status2) is False:
                cell[2] = dir1[i]
                chld.append(cell)
    return chld

def approach3(start, envn, door_pos1,door_pos2):
    chld = []

    if envn.grid.get(door_pos1[0], door_pos1[1]).type == 'wall':
        door_status1 = False
    else:
        door_status1 = envn.grid.get(door_pos1[0], door_pos1[1]).is_locked
    if envn.grid.get(door_pos2[0], door_pos2[1]).type == 'wall':
        door_status2 = False
    else:
        door_status2 = envn.grid.get(door_pos2[0], door_pos2[1]).is_locked

    s = np.array(start)
    for i in range(len(move1)):
        cell = s+move1[i]
        if envn.grid.get(cell[0],cell[1]) is None:
            cell[2] = dir1[i]
            chld.append(cell)
        elif envn.grid.get(cell[0],cell[1]).type != 'wall':
            if door_block(cell, door_pos1, door_status1) is False and door_block(cell, door_pos2, door_status2) is False:
                cell[2] = dir1[i]
                chld.append(cell)
    return chld

# Names of all the problems of part A for which the gifs has to be generated
grids = ['doorkey-5x5-normal',
        'doorkey-6x6-normal',
        'doorkey-8x8-normal',
        'doorkey-6x6-direct',
        'doorkey-8x8-direct',
        'doorkey-6x6-shortcut',
        'doorkey-8x8-shortcut',
        'example-8x8']

# Main function making use of the DP (LCA) written in utils.py -- for par A
def doorkey_problem(env, info):
    '''
    You are required to find the optimal path in
        doorkey-5x5-normal.env
        doorkey-6x6-normal.env
        doorkey-8x8-normal.env
        
        doorkey-6x6-direct.env
        doorkey-8x8-direct.env
        
        doorkey-6x6-shortcut.env
        doorkey-8x8-shortcut.env
        
    Feel Free to modify this fuction
    '''
    door = np.array([info['door_pos'][0], info['door_pos'][1], 0])

    agent = np.array([env.agent_pos[0], env.agent_pos[1], dirr[tuple(env.dir_vec)]])
    goal = np.array([info['goal_pos'][0], info['goal_pos'][1], 0])

    key = np.array([info['key_pos'][0], info['key_pos'][1], 0])

    key_cells = approach(key, env, door)
    door_cells = approach(door, env, door)
    goal_cells = approach(goal, env, door)

    
    def optm(start, cells, locked = True):
        opt_path = list(np.ones((env.height*env.width*4))*MF)
        pick = 0
        if len(cells) != 0:
            for i in cells:
                o, p = dp(start, i, env, door, locked)
                if len(opt_path) > len(o):
                    opt_path = o
                    pick = p
            #opt_path+[MF]
            return list(opt_path), pick
        else:
            return list(np.ones((env.height*env.width*4))*MF), start
    
    path1, pick1 = optm(agent, goal_cells, locked = True)
    path1.append(MF)
    
    #complicated path
    p1,pk1 = optm(agent, key_cells, locked = True)
    #p1.append(MF)
    p2,pk2 = optm(pk1, door_cells, locked = False)
    p3,pk3 = optm(pk2, [goal], locked = False)

    path2 = p1 + [PK] + p2 + [UD] + p3 
    #return path2
    if len(path1)<len(path2):
        return path1
    else:
        return path2

# Main function making use of the DP (LCA) written in utils.py -- for par B
def doorkey_problem2(env, info):
    ''' 
    Feel Free to modify this fuction
    '''

    def optm(start, cells, locked = True):
        opt_path = list(np.ones((env.height*env.width*4))*MF)
        pick = 0
        if len(cells) != 0:
            for i in cells:
                o, p = dp(start, i, env, door, locked)
                if len(opt_path) > len(o):
                    opt_path = o
                    pick = p
            #opt_path+[MF]
            return list(opt_path), pick
        else:
            return list(np.ones((env.height*env.width*4))*MF), start
    
    def optm2(start, cells,door11, door22, locked = True):
        opt_path = list(np.ones((env.height*env.width*4))*MF)
        pick = 0
        if len(cells) != 0:
            for i in cells:
                o, p = dp2(start, i, env, door11, door22, locked)
                if len(opt_path) > len(o):
                    opt_path = o
                    pick = p
            #opt_path+[MF]
            return list(opt_path), pick
        else:
            return list(np.ones((env.height*env.width*4))*MF), start
    
    def optm3(start, cells, door, locked = True):
        opt_path = list(np.ones((env.height*env.width*4))*MF)
        pick = 0
        if len(cells) != 0:
            for i in cells:
                o, p = dp3(start, i, env, locked=False)
                if len(opt_path) > len(o):
                    opt_path = o
                    pick = p
            return list(opt_path), pick
        else:
            return list(np.ones((env.height*env.width*4))*MF), start
    
    door1, door2 = info['door_pos'][0], info['door_pos'][1]

    ds1 = env.grid.get(door1[0], door1[1]).is_locked
    ds2 = env.grid.get(door2[0], door2[1]).is_locked

    # Both doors open
    if ds1 == False and ds2 == False:
        which = 1

    # door1 open
    elif ds1 == False and ds2 == True:
        which = 2
        door = d2tod3(door2)
    
    # door2 open
    elif ds1 == True and ds2 == False:
        which = 3
        door = d2tod3(door1)

    # Both closed
    else:
        which = 4

    agent = np.array([env.agent_pos[0], env.agent_pos[1], dirr[tuple(env.dir_vec)]])
    goal = np.array([info['goal_pos'][0], info['goal_pos'][1], 0])
    key = np.array([info['key_pos'][0], info['key_pos'][1], 0])

    # Correctt
    if which ==2 or which == 3:
        key_cells = approach(key, env, door)
        goal_cells = approach(goal, env, door)
        door_cells = approach(door, env, door)
        path1, pick1 = optm(agent, goal_cells, locked = False)
        path1.append(MF)
        #return path1
        #complicated path
        p1,pk1 = optm(agent, key_cells, locked = True)
        #p1.append(MF)
        p2,pk2 = optm(pk1, door_cells, locked = True)
        p3,pk3 = optm(pk2, [goal], locked = False)

        path2 = p1 + [PK] + p2 + [UD] + p3
        #return path2
        if len(path1)<len(path2):
            return path1
        else:
            return path2

    # Correct
    elif which ==1:

        goal_cells = approach2(goal, env, [0,0,0],[0,0,0])

        path1, pick1 = optm3(agent, goal_cells, door = [0,0,0], locked = True)
        path1.append(MF)
        return path1

    # Correct
    elif which == 4:
        door_cells = []
        #doors = door_unlock(info['door_pos'],env)
        drs = info['door_pos']
        for i in drs:
            if len(i) != 5:
                door_status = env.grid.get(i[0], i[1]).is_locked
                i = d2tod3(i)
                dc = approach2(i, env, drs[0],drs[1])
                for j in dc:
                    if len(j) != 5:
                        door_cells.append(j)
        
        drs[0] = d2tod3(drs[0])
        drs[1] = d2tod3(drs[1])
        key_cells = approach2(key, env, drs[0],drs[1])
        goal_cells = approach2(goal, env, drs[0],drs[1])


        p1,pk1 = optm2(agent, key_cells,drs[0],drs[1], locked = True)
        p2,pk2 = optm2(pk1, door_cells,drs[0],drs[1], locked = True)
        p3,pk3 = optm2(pk2, [goal],drs[0],drs[1], locked = False)

        path2 = p1 + [PK] + p2 + [UD] + p3
        return path2
    
def partA():
    for i in grids:
        env_path = '/Users/orish/sp22/276B/ECE276B_PR1/starter_code/envs/{name}.env'.format(name = i)
        env, info = load_env(env_path) # load an environment

        seq = doorkey_problem(env, info) # find the optimal action sequence
        path='/Users/orish/sp22/276B/ECE276B_PR1/starter_code/gif/ {name} .gif'.format(name = i)
        draw_gif_from_seq(seq, load_env(env_path)[0], path) # draw a GIF & save

def partB():
    env_folder = '/Users/orish/sp22/276B/ECE276B_PR1/starter_code/envs/random_envs'
    env, info, env_path = load_random_env(env_folder)
    seq = doorkey_problem2(env, info) # find the optimal action sequence
    draw_gif_from_seq(seq, load_env(env_path)[0], path='/Users/orish/sp22/276B/ECE276B_PR1/starter_code/gif/doorkey.gif') # draw a GIF & save

if __name__ == '__main__':
    partA()
    #partB()

        
        
    
