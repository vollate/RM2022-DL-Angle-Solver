from ctypes import Union
import math
import numpy as np
import glm
from pandas import date_range


class Time:
    ObservationUpdateTime = 0.05
    VerifyUpdateTime = 0.005

    MaxObservationTime = 10
    MaxVerifyTime = 2

    TrainTime = 10000000


class Const:
    PrintOut=False
    
    CarLength = 0.57
    CarWidth = 0.43
    CarVectorList = [glm.vec3(CarLength / 2, 0, 0),
                     glm.vec3(0, 0, CarWidth / 2),
                     glm.vec3(-CarLength / 2, 0, 0),
                     glm.vec3(0, 0, -CarWidth / 2)]
    InitialBulletTransform = glm.vec3(0, 0, 0)

    Gravity = 9.8
    BulletMass = 0.0032
    GulletDragCoefficient = 0.47
    AirDensity = 1.293
    FrontalArea = 0.001385442  # 42mm bullet
    AirDrugForceConstance = 0.5 * GulletDragCoefficient * AirDensity * FrontalArea

    BulletSpeedRange = [15.0, 30.0]
    IMU_SpeedRange = [-0.2, 0.2]
    IMU_VerticalSpeedRange = [0, 0]
    HostileSpeedRange = [-0.2, 0.2]
    HostileVerticalSpeedRange = [0, 0]
    AngularSpeedRange = [-math.pi, math.pi]
    DistanceRange = [0.7, 5.0]


def r_init(data_range,negative_range=False) -> float:  # random init
    if negative_range:
        return np.random.uniform(data_range[0], data_range[1])*np.random.choice([-1,1])
    return np.random.uniform(data_range[0], data_range[1])
