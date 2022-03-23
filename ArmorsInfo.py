from operator import index
import glm
import math

import ArmorsInfo
from Config import Const, Time,r_init


def modulus_length(vector: glm.vec3, no_root: bool = False):
    root = 1 if no_root else 0.5
    return math.pow(math.pow(vector[0], 2) + math.pow(vector[1], 2) + math.pow(vector[2], 2), root)


def beyond_range(data_range, vector: glm.vec3) -> bool:
    sign = False
    for i in range(3):
        sign = not (data_range[0] <= vector[i] <= data_range[1])
    return sign


class Armors:

    def __init__(self, transform:glm.vec3,first_time=True):
        if first_time:
            self.length = Const.CarLength
            self.width = Const.CarWidth
        self.transform = transform
        self.total_time = 0.0
        self.move_speed=glm.vec3(r_init(Const.HostileSpeedRange),r_init(Const.HostileVerticalSpeedRange),r_init(Const.HostileSpeedRange))
        self.origin_vector_list = Const.CarVectorList
        self.transformed_vector_list = [glm.vec3, glm.vec3, glm.vec3, glm.vec3]
        for i in range(len(self.origin_vector_list)):
            self.transformed_vector_list[i] = self.origin_vector_list[i] + transform
        self.temp_origin_vector_list = None

    def reset(self, transform:glm.vec3):
        self.transform = transform
        self.__init__(transform,False)

    def update_armors(self, angular_speed: float, imu_speed: glm.vec3) -> bool:
        self.total_time += Time.ObservationUpdateTime
        self.transform += Time.ObservationUpdateTime * (self.move_speed-imu_speed)
        if Time.MaxObservationTime < self.total_time or ArmorsInfo.beyond_range(Const.DistanceRange,
                                                                                self.transform):
            return True
        else:
            for i in range(len(self.transformed_vector_list)):
                self.transformed_vector_list[i] = glm.rotateY(self.origin_vector_list[i],
                                                          angular_speed * self.total_time) + self.transform
            return False

    def verify_update_armors(self, angular_speed):
        for i in range(len(self.temp_origin_vector_list)):
            self.temp_origin_vector_list[i] = glm.rotateY(self.temp_origin_vector_list[i], angular_speed * Time.VerifyUpdateTime)
        return self.temp_origin_vector_list

    def verify(self, angular_speed: float, initial_bullet_position: glm.vec3,
               initial_bullet_speed: glm.vec3):#todo: 加入变速小陀螺(画个饼先)
        self.temp_origin_vector_list = self.origin_vector_list[:]

        passed_verify_time = 0.0
        min_distance = modulus_length(self.temp_origin_vector_list)#todo：fix this fucking stupid error
        bullet_position = glm.vec3(initial_bullet_position)
        bullet_speed = initial_bullet_speed
        while passed_verify_time < Time.MaxVerifyTime:
            bullet_position += Time.VerifyUpdateTime * bullet_speed
            net_acceleration = Const.Gravitsssy - Const.AirDrugForceConstance * modulus_length(bullet_speed,
                                                                                            True) / Const.BulletMass
            current_distance = modulus_length(
                bullet_position - self.verify_update_armors(angular_speed))#todo:fix
            if current_distance < min_distance:
                min_distance = current_distance
            else:
                break
            bullet_speed += Time.VerifyUpdateTime * net_acceleration
            passed_verify_time += Time.VerifyUpdateTime
        print(min_distance)
        print("end")
        return 150 * (0.1 - min_distance)

    def get_closest_armor(self,vector_list) -> glm.vec3:
        min_distance = modulus_length(vector_list[1])
        sign = -1
        for i in range(1, 4):
            tem_distance = modulus_length(vector_list[i])
            if tem_distance < min_distance:
                sign = i
                min_distance = tem_distance
        if sign != -1:
            return vector_list[sign]
        else:
            return glm.vec3()
