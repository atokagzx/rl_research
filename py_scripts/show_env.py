# -*- coding: utf-8 -*-
#!/usr/bin/python3

from time import sleep
import argparse
import numpy as np
import os
import shutil
screencast = False
headless = False
# Instantiate the parser
parser = argparse.ArgumentParser(description='Enjoyer optional arguments')

parser.add_argument('--headless', 
                    action='store_true',
                    help="do not use bullet GUI", 
                    default=False)
parser.add_argument('--screencast', 
                    action='store_true',
                    help="capture video from bullet GUI", 
                    default=False)

def screencast(func):
    def wrapper(episode_id, headless):
        if screencast and not headless:
            if episode_id == 0:
                try:
                    shutil.rmtree(f"./screencast/{args.skill}")
                except OSError as e:
                    pass
                os.mkdir(f"./screencast/{args.skill}")
            filename = f"./screencast/{args.skill}/{args.algo}_{str(episode_id)}.mp4"
            print(f'logging into "{filename}"')
            pybullet.startStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4, filename)
            out = func(episode_id, headless)
            pybullet.stopStateLogging(pybullet.STATE_LOGGING_VIDEO_MP4)
        else:
            out = func(episode_id, headless)
        return out
    return wrapper

@screencast
def play_episode(episode_id, headless = False):
    obs = env.reset()
    episode_reward = 0
    episode_length = 0
    for i in range(50):
        # generate random action [-1, 1]
        obs, reward, done, info = env.step((np.random.random(3) -.5) * 2)
        is_success = info['is_success']
        episode_reward += reward
        episode_length += 1
        dt = 1.0 / 120.0
        print("is_success: {}, reward: {}, episode_reward: {}, episode_length: {}".format(is_success, reward, episode_reward, episode_length))
        print("observation:\n{}".format(obs))
        if done:
            break
        if not headless:
            sleep(dt)

if __name__ == "__main__":
    args = parser.parse_args()
    headless = args.headless
    screencast = args.screencast
    if screencast:
        import pybullet
    print("-----------------------------------------\n")
    print("viz     :", not args.headless)
    print("scrcast :", args.screencast)
    print("\n-----------------------------------------")
    from envs.reach import Env as ENV
    env = ENV(headless)
    try:
        # Use deterministic actions for evaluation
        episode_id = 0
        while True:
            play_episode(episode_id, headless)
            episode_id += 1
    except KeyboardInterrupt:
        pass
    # Close process
    env.close()