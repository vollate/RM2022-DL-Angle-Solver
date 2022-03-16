import glm


class Time:
    ObservationUpdateTime = 0.05
    VerifyUpdateTime = 0.001

    MaxObservationTime = 10
    MaxVerifyTime = 2

    TrainTime = 10000


class Const:
    CarLength = 0.57
    CarWidth = 0.43
    CarVectors = [glm.vec3(CarLength / 2, 0, 0),
                  glm.vec3(0, CarWidth / 2, 0),
                  glm.vec3(-CarLength / 2, 0, 0),
                  glm.vec3(0, -CarWidth / 2, 0)]

    Gravity = 9.8
    BulletMass = 1.1  # todo
    GulletDragCoefficient = 0.47
    AirDensity = 1.293
    FrontalArea = 0.001385442  # 42mm bullet
    AirDrugForceConstance = 0.5 * GulletDragCoefficient * AirDensity * FrontalArea

    BulletSpeedRange = [15.0, 30.0]
    IMU_SpeedRange = [-2.0, 2.0]
    AngularSpeedRange = [-1.0, 1.0]
    DistanceRange = [-8.0, 8.0]

    SavePath = "test.pt"
