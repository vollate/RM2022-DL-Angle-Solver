import math

from stable_baselines3 import A2C, DDPG, DQN, PPO
from AimingEnv import AimingEnv, Time, Const

path = "test_ppo_1.pt"
Time.TrainTime=10000000
ALG=PPO
env = AimingEnv()

model = ALG('MlpPolicy', env, verbose=1)
model.learn(total_timesteps=Time.TrainTime)
model.save(path)

model = ALG.load(path)
obs = env.reset()
Const.PrintOut=True
for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()
