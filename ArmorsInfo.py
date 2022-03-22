from operator import index
import glm
import math

import ArmorsInfo
from Config import Const, Time


def modulus_length(vector: glm.vec3, no_root: bool = False):
    root = 1 if no_root else 0.5
    return math.pow(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2), root)


def beyond_range(data_range, vector: glm.vec3) -> bool:
    sign = False
    for i in range(3):
        sign = not (data_range[0] <= vector[i] <= data_range[1])
    return sign


class Armors:
    total_time = 0.0
    transform = None
    origin_vectors = None

    def __init__(self, transform):
        self.length = Const.CarLength
        self.width = Const.CarWidth
        self.transform = transform
        self.total_time = 0.0
        self.origin_vectors = Const.CarVectors
        self.transformed_vectors = [glm.vec3, glm.vec3, glm.vec3, glm.vec3]
        for i in range(len(self.origin_vectors)):
            self.transformed_vectors[i] = self.origin_vectors[i] + transform
        self.verify_vectors = None

    def reset(self, transform):
        self.transform = transform
        self.__init__(transform)

    def update_armors(self, angular_speed: float, imu_speed: glm.vec3) -> bool:
        self.total_time += Time.ObservationUpdateTime
        self.transform -= Time.ObservationUpdateTime * imu_speed
        if Time.MaxObservationTime < self.total_time or ArmorsInfo.beyond_range(Const.DistanceRange,
                                                                                self.transform):
            return True
        else:
            for i in range(len(self.transformed_vectors)):
                self.transformed_vectors[i] = glm.rotateY(self.origin_vectors[i],
                                                          angular_speed * self.total_time) + self.transform
            return False

    def verify_update_armors(self, angular_speed):
        self.verify_vectors = glm.rotateY(self.verify_vectors, angular_speed * Time.VerifyUpdateTime)
        return self.verify_vectors

    def verify(self, angular_speed: float, initial_bullet_position: glm.vec3,
               initial_bullet_speed: glm.vec3):
        self.verify_vectors = self.get_closest_armor()#copy(self.origin_vectors)#todo
        # print(self.verify_vectors)
        passed_verify_time = 0.0
        min_distance = modulus_length(self.verify_vectors)
        # print(min_distance)
        bullet_position = glm.vec3(initial_bullet_position)
        bullet_speed = initial_bullet_speed
        while passed_verify_time < Time.MaxVerifyTime:
            bullet_position += Time.VerifyUpdateTime * bullet_speed
            net_acceleration = Const.Gravity - Const.AirDrugForceConstance * modulus_length(bullet_speed,
                                                                                            True) / Const.BulletMass
            current_distance = modulus_length(
                bullet_position - self.verify_update_armors(angular_speed))
            if current_distance < min_distance:
                min_distance = current_distance
            else:
                break
            bullet_speed += Time.VerifyUpdateTime * net_acceleration
            passed_verify_time += Time.VerifyUpdateTime
        print(min_distance)
        print("end")
        return 150 * (0.1 - min_distance)

    def get_closest_armor(self) -> glm.vec3:
        min_distance = modulus_length(self.transformed_vectors[1])
        sign = -1
        for i in range(1, 4):
            tem_distance = modulus_length(self.transformed_vectors[i])
            if tem_distance < min_distance:
                sign = i
                min_distance = tem_distance
        if sign != -1:
            return self.transformed_vectors[sign]
        else:
            return glm.vec3()
