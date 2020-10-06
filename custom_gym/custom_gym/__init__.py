from gym.envs.registration import register

register(
    id='RLQuad-v0',
    entry_point='custom_gym.envs:RLQuad',
    # max_episode_steps=300,
    # reward_threshold=1000.0,
)