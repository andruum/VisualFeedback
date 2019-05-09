
import matplotlib.pyplot as plt
import re

import numpy as np
from math import radians, degrees
from numpy.random.mtrand import randn


def parsefile(file):
    lineList = [line.rstrip('\n') for line in open(file)]
    results_list = []
    patern = "(\[|,)*([^,\[\]]+)(\]|,)*"
    for line in lineList:
        result_line = []
        strings = re.findall(patern, line)
        for s in strings:
            result_line.append(float(s[1]))
        results_list.append(result_line)
    return results_list

LINE_STYLES = ['-', '--', '-.', ':',  (5, (4, 2)) ]

def plot(robot, estimation):

    robot = np.asarray(robot)
    plt.plot(robot[:,-1], robot[:, 0], label="Реальное положение")

    estimation = np.asarray(estimation)

    for i in range(len(estimation[0])-1):
        plt.plot(estimation[:, -1], estimation[:, i], label="Положение звена " + str(i+1), linestyle=LINE_STYLES[i])

    plt.xlabel('Время, с')
    plt.ylabel('Угол, град')
    plt.title('Определение положения звеньев')

    plt.legend(loc="upper right")
    # plt.show()
    plt.savefig('state_estimation_modeled.png')

def make_filter_bytime(states, start_time, end_time):
    def filter_func(measure):
        return start_time < measure[states] < end_time
    return filter_func

def make_time_scale(states, starttime_nonnormalized, starttime, timescale):
    def func(x):
        x[0:states] = map(lambda x:degrees(x),x[0:states])
        x[states] = (x[states] - starttime_nonnormalized - starttime) * timescale
        return x
    return func

def extract_results(results_with_time, starttime, endtime, timescale):
    states = len(results_with_time[0])-1
    starttime_nonnormalized = results_with_time[0][states]

    filter_func = make_filter_bytime(states, starttime_nonnormalized+starttime, starttime_nonnormalized+endtime)
    results_filtered = list(filter(filter_func,results_with_time))

    mapfunc = make_time_scale(states, starttime_nonnormalized, starttime, timescale)
    results_filtered = list(map(mapfunc, results_filtered))
    return results_filtered


def normalize_to_angle(results, angle):
    results = np.asarray(results)

    for i in range(len(results[0])-1):
        mean = np.mean(results[:,i])
        results[:,i] -= mean

    return results


def model_with_noise(estimations, ground_truth_src):
    ground_truth_src = np.asarray(ground_truth_src)
    ground_truth = ground_truth_src[::30, :]
    modeled_estimation = np.zeros((len(ground_truth), len(ground_truth[0])))
    std = np.std(estimations, axis=0)

    for i in range(len(estimations[0])-1):
        modeled_noise = np.random.normal(0, std[i], len(ground_truth))
        modeled_estimation[:,i] = np.add(ground_truth[:, i], modeled_noise)
    modeled_estimation[:, -1] =  ground_truth[:,-1]
    return modeled_estimation

FILE_ROBOT = "../robot/20190509185842925875.txt"
FILE_ESTIMATED = "../20190509155905902671_estimated.txt"

if __name__ == '__main__':
    estimated = parsefile(FILE_ESTIMATED)
    robot = parsefile(FILE_ROBOT)

    estimated = extract_results(estimated, 1, 25, 1.0)
    robot = extract_results(robot, 0, 25, 1.0)

    estimated = model_with_noise(estimated,robot)

    plot(robot, estimated)