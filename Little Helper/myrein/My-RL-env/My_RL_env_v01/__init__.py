from gym.envs.registration import register

register(
    id='My_RL-v0',
    entry_point='My_RL_env_v01.envs:ReinEnv',
)
