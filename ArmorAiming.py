from typing import Optional, Union
import numpy as np
import glm
import math
import gym
from typing import Optional, Union
from gym import logger, spaces
from gym.utils import seeding
from ArmorsInfo import Armors, Const, Time,r_init


class AimingEnv(gym.Env):

    def __init__(self):
        self.gravitational_acceleration = glm.vec3(0, Const.Gravity, 0)
        self.angular_speed = r_init(Const.AngularSpeedRange)
        self.bullet_speed = glm.vec3(r_init(Const.BulletSpeedRange), 0, 0)
        self.observation_env = None
        self.imu_speed = glm.vec3(r_init(Const.IMU_SpeedRange), r_init(Const.IMU_VerticalSpeedRange),
                                  r_init(Const.IMU_SpeedRange))
        self.hostile_armors = Armors(glm.vec3(r_init(Const.DistanceRange), 0, r_init(Const.DistanceRange)))

        self.action_space = spaces.Box(low=np.array([-math.pi, -math.pi]), high=np.array([math.pi, math.pi]),
                                       dtype=float)
        self.observation_space = spaces.Box(
            low=np.array([Const.DistanceRange[0], 0, Const.DistanceRange[0],
                          Const.IMU_SpeedRange[0], 0, Const.IMU_SpeedRange[0], Const.BulletSpeedRange[0]]),
            high=np.array([Const.DistanceRange[1], 0, Const.DistanceRange[1],
                           Const.IMU_SpeedRange[1], 0, Const.IMU_SpeedRange[1], Const.BulletSpeedRange[1]]),
            dtype=float)

    def step(self, action):

        yaw = action[0]
        pitch = action[1]
        initial_imu_speed = glm.vec3(self.observation_env[3], self.observation_env[4], self.observation_env[5])
        initial_bullet_speed = glm.rotateX(glm.rotateY(self.bullet_speed, yaw), pitch) + initial_imu_speed

        reward = self.hostile_armors.verify(self.angular_speed, Const.InitialBulletTransform,
                                            initial_bullet_speed)
        done = self.hostile_armors.update_armors(self.angular_speed,
                                                 self.imu_speed)
        temp_vector = self.hostile_armors.get_closest_armor()
        self.observation_env = [temp_vector.x, temp_vector.y, temp_vector.z, self.imu_speed.x, self.imu_speed.y,
                                self.imu_speed.z, r_init(Const.BulletSpeedRange)]
        return np.array(self.observation_env, dtype=float), reward, done, {}

    def reset(
            self,
            *,
            seed: Optional[int] = None,
            return_info: bool = False,
            options: Optional[dict] = None,
    ):
        # super().reset(seed=seed)
        self.hostile_armors.reset(glm.vec3(r_init(Const.DistanceRange), 0, r_init(Const.DistanceRange)))
        self.imu_speed = glm.vec3(r_init(Const.IMU_SpeedRange), 0, r_init(Const.IMU_SpeedRange))
        temp_vector = self.hostile_armors.get_closest_armor()#todo:fix
        self.angular_speed = r_init(Const.AngularSpeedRange)
        self.bullet_speed = glm.vec3(r_init(Const.BulletSpeedRange),0,0)
        self.observation_env = np.array(
            [temp_vector.x, temp_vector.y, temp_vector.z, self.imu_speed.x, self.imu_speed.y, self.imu_speed.z,
             r_init(Const.BulletSpeedRange)],
            dtype=float)
        if not return_info:
            return self.observation_env
        else:
            return self.observation_env, {}

   

    def render(self, mode="human"):
        return None

    def close(self):
        return None
