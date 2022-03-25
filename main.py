from stable_baselines3 import PPO

from ArmorAiming import AimingEnv, Time, Const

env = AimingEnv()

# while True:
#     env.reset()
#     a = 1

model = PPO('MlpPolicy', env, verbose=1, device="cpu")
model.learn(total_timesteps=Time.TrainTime)
model.save(Const.SavePath)
obs = env.reset()

for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = env.step(action)
    env.render()
    if done:
        obs = env.reset()
