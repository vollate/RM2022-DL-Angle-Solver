import math

from stable_baselines3 import A2C, DDPG, DQN, PPO
import glm
from ArmorAiming import AimingEnv, Time, Const

path = "test.pt"
env = AimingEnv()

model = PPO('MlpPolicy', env, verbose=1, device="auto")
model.learn(total_timesteps=Time.TrainTime)
model.save(path)

model = PPO.load(path)
obs = env.reset()

for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()
