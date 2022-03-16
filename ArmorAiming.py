from typing import Optional, Union
import numpy as np
import glm
import math
import gym
from typing import Optional, Union
from gym import logger, spaces
from gym.utils import seeding
from ArmorsInfo import Armors, Const, Time


class AimingEnv(gym.Env[np.float, np.float]):

    def __init__(self):
        self.gravitational_acceleration = glm.vec3(0, Const.Gravity, 0)
        self.angular_speed = self.r_init(Const.AngularSpeedRange)
        self.bullet_speed = glm.vec3(self.r_init(Const.BulletSpeedRange), 0, 0)
        self.observation_env = None
        self.imu_speed = glm.vec3(self.r_init(Const.IMU_SpeedRange), 0, self.r_init(Const.IMU_SpeedRange))
        self.hostile_armors = Armors(glm.vec3(self.r_init(Const.DistanceRange), 0, self.r_init(Const.DistanceRange)))

        self.action_space = spaces.Box(low=np.array([-math.pi, -math.pi]), high=np.array([math.pi, math.pi]),
                                       dtype=float)
        self.observation_space = spaces.Box(
            low=np.array([Const.DistanceRange[0], 0, Const.DistanceRange[0],
                          Const.IMU_SpeedRange[0], 0, Const.IMU_SpeedRange[0]]),
            high=np.array([Const.DistanceRange[1], 0, Const.DistanceRange[1],
                           Const.IMU_SpeedRange[1], 0, Const.IMU_SpeedRange[1]]), dtype=float)

    def step(self, action):

        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.observation_env is not None, "Call reset before using step method."

        yaw = action[0]
        pitch = action[1]
        initial_imu_speed = glm.vec3(self.observation_env[3], self.observation_env[4], self.observation_env[5])
        initial_bullet_speed = glm.rotateX(glm.rotateY(self.bullet_speed, yaw), pitch) + initial_imu_speed
        initial_bullet_position = glm.vec3(0, 0, 0)
        reward = self.hostile_armors.verify(self.angular_speed, initial_bullet_position,
                                            initial_bullet_speed)
        done = self.hostile_armors.update_armors(self.angular_speed,
                                                 self.imu_speed)
        temp_vector = self.hostile_armors.get_closest_armor()
        self.observation_env = [temp_vector.x, temp_vector.y, temp_vector.z, self.imu_speed.x, self.imu_speed.y,
                                self.imu_speed.z]
        return np.array(self.observation_env, dtype=float), reward, done, {}

    def reset(
            self,
            *,
            seed: Optional[int] = None,
            return_info: bool = False,
            options: Optional[dict] = None,
    ):
        super().reset(seed=seed)
        self.hostile_armors.reset(glm.vec3(self.r_init(Const.DistanceRange), 0, self.r_init(Const.DistanceRange)))
        self.imu_speed = glm.vec3(self.r_init(Const.IMU_SpeedRange), 0, self.r_init(Const.IMU_SpeedRange))
        temp_vector = self.hostile_armors.get_closest_armor()
        self.observation_env = np.array(
            [temp_vector.x, temp_vector.y, temp_vector.z, self.imu_speed.x, self.imu_speed.y, self.imu_speed.z],
            dtype=float)
        if not return_info:
            return self.observation_env
        else:
            return self.observation_env, {}

    @staticmethod
    def r_init(data_range) -> float:  # random init
        return np.random.uniform(data_range[0], data_range[1])

    def render(self, mode="human"):
        return None

    def close(self):
        return None
