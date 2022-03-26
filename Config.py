import math
import numpy as np
import glm


class Time:
    ObservationUpdateTime = 0.05
    VerifyUpdateTime = 0.005

    MaxObservationTime = 10
    MaxVerifyTime = 2

    TrainTime = 1000000


class Const:
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
    IMU_SpeedRange = [-0.5, 0.5]
    IMU_VerticalSpeedRange = [0, 0]
    HostileSpeedRange = [-0.5, 0.5]
    HostileVerticalSpeedRange = [0, 0]
    AngularSpeedRange = [-math.pi, math.pi]
    DistanceRange = [-5.0, 5.0]


def r_init(data_range) -> float:  # random init
    return np.random.uniform(data_range[0], data_range[1])
