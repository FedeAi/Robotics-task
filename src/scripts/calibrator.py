from math import sqrt

import optuna
import pandas as pd
import models
import numpy as np


def pi2pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def computeRobotError(w, l, r, CPR):
    distance_error = 0
    angle_error = 0

    robot = models.Robot(w, l, r, 5.0, CPR, startingPose)
    numIterations = 0
    for i, row in bag.iterrows():
        pose = robot.updatePose(row["eFL"], row["eFR"], row["eRR"], row["eRL"], row["timestamp"])

        distance_error += (row["x"] - pose[1]) ** 2 + (row["y"] - pose[2]) ** 2
        angle_error += pi2pi(pi2pi(row["theta"]) - pi2pi(pose[0])) ** 2
        numIterations = i
    distance_error /= numIterations
    angle_error /= numIterations

    return distance_error + angle_error


def objective(trial):
    w = trial.suggest_float('w', 0.160, 0.178)
    l = trial.suggest_float('l', 0.19, 0.21)
    r = trial.suggest_float('r', 0.065, 0.075)
    CPR = trial.suggest_int("CPR", 40, 44, step=1)
    return computeRobotError(w, l, r, CPR)


if __name__ == '__main__':
    bag = pd.read_csv("bags/bag2.csv")
    startingPose = bag[["theta", "x", "y"]].iloc[0].values

    baseLineError = computeRobotError(0.168, 0.195, 0.0725, 40)
    print("BASE LINE ERROR: %f", baseLineError)
    study = optuna.create_study()
    study.optimize(objective, n_trials=200, n_jobs=8)

    optuna.visualization.plot_parallel_coordinate(study).show(renderer="browser")
    optuna.visualization.plot_contour(study).show(renderer="browser")
    optuna.visualization.plot_param_importances(
        study, target=lambda t: t.duration.total_seconds(), target_name="duration"
    ).show(renderer="browser")

    ##
    # robot = models.Robot(0.169, 0.2, 0.07, 5.0, 42.0, startingPose)
    # distance_error = 0
    # # discard_factor = 0
    # for i, row in bag.iterrows():
    #     pose = robot.updatePose(row["eFL"], row["eFR"], row["eRR"], row["eRL"], row["timestamp"])
    #
    #     distance_error += sqrt((row["x"] - pose[1]) ** 2 + (row["y"] - pose[2]) ** 2)
    #
    #     # print("theta: %f - %f", row["theta"],  pose[0])
    #     # print("x: %f - %f", row["x"],  pose[1])
    #     # print("y: %f - %f", row["y"],  pose[2])
    #     # print("#"* 30)
    # print(distance_error)

"""

"""
