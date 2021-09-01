from env import *
from quadrotor_dynamics import *
from transformations import *

params=CopterParams()
env=QuadRotorEnv(params)
env.reset()
for i in range(100):
  actions=np.random.random_sample((4,))
  state, reward, done, info=env.step(actions)
  print(state)