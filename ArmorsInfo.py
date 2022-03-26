from operator import index
from black import out
import glm
import math
from typing import List, Union

import ArmorsInfo
from Config import Const, Time, r_init


def modulus_length(vector: glm.vec3, square: bool = False) -> float:
    root = 1 if square else 0.5
    return math.pow(math.pow(vector.x, 2) + math.pow(vector.y, 2) + math.pow(vector.z, 2), root)


def unit_vector(vector: glm.vec3) -> glm.vec3:
    return vector / modulus_length(vector)


def beyond_range(data_range: Union[list, float], vector: glm.vec3) -> bool:
    return not (data_range[0] <= vector[2] <= data_range[1]) or not (data_range[0] <= vector[0] <= data_range[1]) or not (-data_range[1]<=vector[0]<=-data_range[0])
 


def get_closest_armor(vector_list: Union[list, glm.vec3]) -> glm.vec3:
    min_distance = modulus_length(vector_list[1])
    sign = 0
    for i in range(1, 4):
        tem_distance = modulus_length(vector_list[i])
        if tem_distance < min_distance:
            min_distance = tem_distance
            sign = i
    return vector_list[sign]


class Armors:

    def __init__(self, angular_speed: float, move_speed: glm.vec3, transform: glm.vec3):
        self.transform = transform
        self.total_time = 0.0
        self.angular_speed = angular_speed
        self.move_speed = move_speed
        self.origin_vector_list = Const.CarVectorList[:]
        self.transformed_vector_list = [glm.vec3, glm.vec3, glm.vec3, glm.vec3]
        for i in range(len(self.origin_vector_list)):
            self.transformed_vector_list[i] = self.origin_vector_list[i] + self.transform

    def reset(self, angular_speed: float, move_speed: glm.vec3, transform: glm.vec3):
        self.__init__(angular_speed, move_speed, transform)

    def update_armors(self, angular_speed: float, move_speed: glm.vec3, imu_speed: glm.vec3) -> bool:
        self.total_time += Time.ObservationUpdateTime
        self.angular_speed = angular_speed
        self.move_speed = move_speed
        self.transform += Time.ObservationUpdateTime * (self.move_speed - imu_speed)
        if Time.MaxObservationTime < self.total_time or ArmorsInfo.beyond_range(Const.DistanceRange,
                                                                                self.transform):
            return True
        else:
            for i in range(len(self.transformed_vector_list)):
                self.origin_vector_list[i] = glm.rotateY(self.origin_vector_list[i],
                                                         self.angular_speed * Time.ObservationUpdateTime)
                self.transformed_vector_list[i] = self.origin_vector_list[i] + self.transform
            return False

    @staticmethod
    def verify_update_armors(temp_origin_vector_list, temp_transformed_vector_list, transform: glm.vec3,
                             hostile_speed: glm.vec3, angular_speed: float):
        transform += Time.VerifyUpdateTime * hostile_speed
        for i in range(len(temp_origin_vector_list)):
            temp_origin_vector_list[i] = glm.rotateY(temp_origin_vector_list[i],
                                                     angular_speed * Time.VerifyUpdateTime)
            temp_transformed_vector_list[i] = temp_origin_vector_list[i] + transform
        return None

    def verify(self, hostile_speed: glm.vec3, angular_speed: float, initial_bullet_position: glm.vec3,
               initial_bullet_speed: glm.vec3):  # todo: 加入变速小陀螺
        temp_origin_vector_list = self.origin_vector_list[:]
        temp_transform = self.transform
        temp_transformed_vector_list = [glm.vec3, glm.vec3, glm.vec3, glm.vec3]
        for i in range(len(temp_transformed_vector_list)):
            temp_transformed_vector_list[i] = temp_origin_vector_list[i] + temp_transform
        min_distance = modulus_length(get_closest_armor(temp_transformed_vector_list))
        passed_verify_time = 0.0
        bullet_position = glm.vec3(initial_bullet_position)
        bullet_speed = glm.vec3(initial_bullet_speed)
        reversed_direction = True

        while passed_verify_time < Time.MaxVerifyTime:
            bullet_position += Time.VerifyUpdateTime * bullet_speed
            bullet_speed += Time.VerifyUpdateTime * (glm.vec3(0, -Const.Gravity, 0) - unit_vector(
                bullet_speed) * Const.AirDrugForceConstance * modulus_length(bullet_speed, True) / Const.BulletMass)
            passed_verify_time += Time.VerifyUpdateTime
            self.verify_update_armors(temp_origin_vector_list, temp_transformed_vector_list, temp_transform,
                                      hostile_speed, angular_speed)

            current_distance = modulus_length(bullet_position - get_closest_armor(temp_transformed_vector_list))
            if current_distance < min_distance:
                min_distance = current_distance
                reversed_direction = False
            else:
                break
        if Const.PrintOut:
            print(min_distance)
            print("end")
        return 1500 * (0.05 - min_distance) if not reversed_direction else -150000 * min_distance
