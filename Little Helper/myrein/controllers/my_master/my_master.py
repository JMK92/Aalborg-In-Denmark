#!/home/jeongmo/anaconda3/envs/myenv/bin/python3.7
import gym
import My_RL_env_v01
#import ReinEnv
import random
import numpy as np
import time
import os

from stable_baselines.common.policies import MlpPolicy #PPO
#from stable_baselines.sac.policies import MlpPolicy # SAC
from stable_baselines import SAC
from stable_baselines import PPO1

def read_file(filename):
    with open(filename) as fp:
        return int(fp.read())

# Make sure to increment this with each run
simRunID = "PPO1_12Lidar"

# Creating file structure
simLogPath = "Logging/"+simRunID+"/"
if not os.path.exists(simLogPath): #Create directories
    os.makedirs(simLogPath + 'models/', exist_ok=True)
    os.makedirs(simLogPath + 'log/', exist_ok=True)
    os.makedirs(simLogPath + 'screenshots/collision', exist_ok=True)
    os.makedirs(simLogPath + 'screenshots/timeout', exist_ok=True)
    os.makedirs(simLogPath + 'screenshots/success', exist_ok=True)
    print('Directory structure created...')
#simRunTime = time.strftime("%Y-%m-%d", time.gmtime()) # File name
simName = 'PPO1_12Lidar'
epFile = simLogPath + 'log/eps.txt'
currentEp = 0
maxEpisodes = 20000
timeStepsPerLoad = 60000
tensorboardPath = simLogPath + 'Myv1_tensorboard/'

env = gym.make('My_RL-v0')
print('Environment created...')
# Check how many episodes the model has been trained for:
if os.path.exists(epFile):
  currentEp = read_file(epFile)
  print('Resuming training from episode {}'.format(currentEp))

#  Check if training log of previous training exists, load model if does
if currentEp <= maxEpisodes:
  if os.path.exists(simLogPath + 'log/P7_NoObstacles.txt'):
    print('Loading previous model...')
    model = PPO1.load(simLogPath +'models/'+ simName, env, tensorboard_log=tensorboardPath)
  else:
    print('Creating new model...')
    model = PPO1(MlpPolicy, env, verbose=1, tensorboard_log=tensorboardPath)
  model.learn(total_timesteps=timeStepsPerLoad, log_interval=1)
  print('Training finished...')
  model.save(simLogPath +'models/'+ simName)
  print('Model saved...')
  env.SaveAndQuit()
else:
  print("Training finished.")
  env.supervisor.simulationSetMode(0)