import matplotlib.pyplot
import matplotlib.pyplot as plt
import random
import math
import copy
import datetime
import pointInPoly
from Tree import tree
import numpy as np


# define the range of map   random sample

# def check_in_the_map(map,node):
#     result=True
#     for robot in node:
#         if map[0][0]<=robot[0] and robot[0]<=map[0][1] and map[1][0]<=robot[1] and robot[1]<=map[1][1]:
#             result=True
#         else:
#             result=False
#     return result

def random_generate(map):
    node_value=[]
    for i in range(robot_number):
        x=random.uniform(map[0][0],map[0][1])
        y=random.uniform(map[1][0],map[1][1])
        node_value.append([x,y])
    random_node=tree(node_value)
    return random_node

def check_collision(state, obstacles):
    for pos in state:
        circle = circle_sampling(pos)
        for obs in obstacles:
            for point in circle:
                if pointInPoly.is_inside_polygon(obs, tuple(point)):
                    return True
    if check_circle_collision(state):
        return True

    return False

def circle_sampling(pos, arc_length = 0.1, radius = 1):
    n = math.ceil((2*radius*math.pi) / arc_length)
    thetas = np.linspace(0, 2*math.pi, n)
    thetas = thetas.reshape(1, -1)
    x = np.cos(thetas) * radius + pos[0]
    y = np.sin(thetas) * radius + pos[1]
    return np.concatenate((x, y)).T

    # theta = np.random.uniform(0, 2 * math.pi, (1, n))
    # r = radius
    # x = np.cos(theta) * r + pos[0]
    # y = np.sin(theta) * r + pos[1]

def check_circle_collision(state, radius = 1):
    for row in range(len(state)):
        temp = np.delete(state, row, axis=0)
        target = state[row]
        # temp = copy.deepcopy(state)
        # temp.remove(target)
        for other in temp:
            if np.linalg.norm(np.array(target) - np.array(other)) <= radius * 2:
                return True
    return False

def rrt_extend_single(Ta, q_rand, step_length):
    q_target = np.array([])
    q_near = find_nearest(Ta, q_rand)
    q_int = limit(q_rand, q_near, step_length)
    result = local_planner(q_near, q_int, step_size)
    if result:
        q_near.add_node(q_int)
        q_target = q_int
    return result, Ta, q_target

def rrt_extend_multiple(Tb, q_target, step_length):
    q_connect = np.array([])
    q_near = find_nearest(Tb, q_target)
    q_int = limit(q_target, q_near, step_length)
    q_last = q_near
    num_steps = math.ceil(np.linalg.norm(q_target.value - q_near.value) / step_length)
    for i in range(num_steps):
        result2 = local_planner(q_int, q_last, step_size)
        if not result2:
            return result2, Tb, q_connect
        q_near.add_node(q_int)
        q_connect = q_int
        if i < num_steps:
            q_last = q_int
            q_int = limit(q_target, q_int, step_length)
    return result2, Tb, q_connect

def local_planner(q_near, q_int, step_size):
    q_near = np.array(q_near.value)
    q_int = np.array(q_int.value)

    delta_q = q_int - q_near
    num_steps = math.ceil(np.linalg.norm(delta_q)/step_size)
    step = delta_q/num_steps
    collision = False
    q = q_near
    for i in range(num_steps):
        q = q + step
        if check_collision(q, obstacles):
            collision = True
            break

    success = not collision
    return success

def limit(q_rand, q_near, step_length):
    q_rand = np.array(q_rand.value)
    q_near = np.array(q_near.value)
    return tree(q_near + (q_rand - q_near) * step_length)

def find_nearest(Ta, q_rand):
    nodes = Ta.travel()
    nodes = np.array(nodes)
    min = 1000
    res = 0
    for node in nodes:
        temp = np.linalg.norm(np.array(node) - np.array(q_rand.value))
        if min > temp:
            min = temp
            res = node
    return Ta.search(res)

def extract_path(t, node):
    result = []
    result.append(node.value)
    current = node
    while current.parent is not None:
        result.append(list(current.parent.value))
        current = current.parent
    return result

def circle(x,y,r):
    th = np.arange(0, 2 * math.pi, math.pi / 50)
    xunit = r * np.cos(th) + x
    yunit = r * np.sin(th) + y
    plt.plot(xunit, yunit)

def plot(qconnect, Ta, Tb):
    nearest_a = find_nearest(Ta, q_connect)
    nearest_b = find_nearest(Tb, q_connect)
    list_a = extract_path(Ta, nearest_a)
    list_b = extract_path(Tb, nearest_b)
    list_a[-1] = np.array(list_a[-1])
    list_b[-1] = np.array(list_b[-1])

    list_a.reverse()
    totally_result = list_a
    totally_result.extend(list_b)
    totally_result = np.array(totally_result)

    path1_x = totally_result[:, 0, 0]
    path1_y = totally_result[:, 0, 1]

    path2_x = totally_result[:, 1, 0]
    path2_y = totally_result[:, 1, 1]

    path3_x = totally_result[:, 2, 0]
    path3_y = totally_result[:, 2, 1]

    plt.ion()

    plt.title("RRT")
    plt.grid(True)
    for index in range(len(path1_x)):
        if index != 0:
            plt.plot(path1_x[index - 1: index + 1], path1_y[index - 1: index + 1], '--')
            plt.plot(path2_x[index - 1: index + 1], path2_y[index - 1: index + 1], '--')
            plt.plot(path3_x[index - 1: index + 1], path3_y[index - 1: index + 1], '--')
        circle(path1_x[index], path1_y[index], 1)
        circle(path2_x[index], path2_y[index], 1)
        circle(path3_x[index], path3_y[index], 1)

        for obs in np.array(obstacles):
            plt.xlim(0, 20)
            plt.ylim(0, 20)
            x = obs[:, 0]
            y = obs[:, 1]
            plt.fill(x, y, 'grey')
        plt.pause(1)

    plt.ioff()
if __name__ == '__main__':
    # step_sizes = np.arange(0.1, 1.1, 0.1)
    # step_lengths = np.arange(0.1, 0.6, 0.1)
    # for step_size in step_sizes:
    # for step_length in step_lengths:
    step_length = 0.5
    step_size = 0.1
    pathLength = 0
    begin_time = datetime.datetime.now()
    sum_performance = 0
    total_num_failure = 0
    trail_num = 1
    #for step_length in step_lengths:
    for i in range(trail_num):
        # start of the algorithm section
        obstacles = [[[0, 0], [8, 0], [8, 4.5], [0, 4.5]], [[20, 0], [12, 0], [12, 4.5], [20, 4.5]]]

        map = [[0, 20], [0, 10]]
        start = [[1.5, 5.5], [1.5, 8.0], [18.5, 7.5]]
        goal = [[18.5, 5.5], [18.5, 8.0], [1.5, 7.5]]
        robot_number = 3

        Ta = tree(start)
        Tb = tree(goal)
        success = False
        MaxNodes = 10000
        performance = 0
        for i in range(MaxNodes):
            q_rand = random_generate(map)
            result, Ta, q_target = rrt_extend_single(Ta, q_rand, step_length)
            if result:
                result2, Tb, q_connect = rrt_extend_multiple(Tb, q_target, step_length)
                if result2:
                    success = True
                    break
            temp = Tb
            Tb = Ta
            Ta = temp
            performance += 1

        # the main algorithm is finished, record the performance
        sum_performance += performance
        # optional plot
        if not success:
            print("mission failed, we ll get em next time")

        else:
            plot(q_connect, Ta, Tb)
            #pass
        # check the num of failure and path length
        nearest_a = find_nearest(Ta, q_connect)
        nearest_b = find_nearest(Tb, q_connect)
        list_a = extract_path(Ta, nearest_a)
        list_b = extract_path(Tb, nearest_b)
        pathLength += len(list_a) + len(list_b) + 1
        list_a.reverse()
        totally_result = list_a
        totally_result.extend(list_b)
        totally_result = np.array(totally_result)
        for test in totally_result:
            if check_circle_collision(test):
                total_num_failure += 1

        # after trials, print the average performance by the num nodes created
        print("step size: {}".format(step_size))
        print("step length: {}".format(step_length))
        print("average runtime: {}".format((datetime.datetime.now() - begin_time) / trail_num))
        print("average performance: {}".format(sum_performance/trail_num))
        print("average path length: {}".format(pathLength/trail_num))
        print("average failure rate: {}".format(total_num_failure / len(totally_result)))









