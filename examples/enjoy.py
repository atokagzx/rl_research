# -*- coding: utf-8 -*-
#!/usr/bin/python3

from time import sleep
import argparse
import numpy as np
import pybullet
from stable_baselines3 import PPO
import ur5_gym
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

parser.add_argument('-m', '--model', 
                    type=str, 
                    help="Path to trained model (default: ./trained_weights/[SKILL]/best_model.zip)", 
                    default=None)

parser.add_argument('-a', '--algo',
                    type=str, 
                    default='PPO',
                    help='[PPO, SAC, TD3] (default: PPO)')

parser.add_argument('-s', '--skill', 
                    type=str, 
                    default='reach',
                    help='[reach, reach_obj, push, slide, pick] (default: reach)')          

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
    obs = enjoy_env.reset()
    done = False
    episode_reward = 0.0
    episode_length = 0
    step = 0
    is_success = False
    while not done:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = enjoy_env.step(action)
        is_success = info['is_success'] # get latest success status
        episode_reward += reward
        episode_length += 1
        step += 1
        dt = 1.0 / 120.0
        if not headless:
            sleep(dt)
    return episode_reward, episode_length, is_success

if __name__ == "__main__":
    
    args = parser.parse_args()
    if args.model == None:
        args.model = "trained_weights/" + args.skill + "/best_model.zip"
    save_path = args.model
    print("-----------------------------------------\n")
    print("skill   :", args.skill)
    print("algo    :", args.algo)
    print("viz     :", not args.headless)
    print("model   :", args.model)
    print("scrcast :", args.screencast)
    print("\n-----------------------------------------")
    
    RL_ALGORITHM = args.algo
    N_ITERATIONS = 1000
    if args is not None:
        headless = args.headless
        screencast = args.screencast
    #choose environment for skill 
    try:
        if args.skill == 'reach': 
            ENV = ur5_gym.ReachEnv
        else:
            raise ImportError
    except ImportError as error:
        print('ERROR: Requested skill "{0}" not implemented yet.'.format(args.skill))
        exit()

    algo = {
        "PPO": PPO,
    }[RL_ALGORITHM]
    enjoy_env = ENV(headless)
    
    try:
        model = algo.load(save_path, env=ENV)
    except FileNotFoundError:
        print("-----------------------------------------\n")
        print('ERROR: File "{0}" does not exist.'.format(save_path))
        print("\n-----------------------------------------")
        exit()

    try:
        # Use deterministic actions for evaluation
        episode_rewards, episode_lengths, episode_success = [], [], []
        for episode_id in range(N_ITERATIONS):
            episode_reward, episode_length, is_success = play_episode(episode_id, headless)
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            episode_success.append(is_success)
            print(
                f"Episode {len(episode_rewards)} reward={episode_reward}, length={episode_length}, is success={str(is_success)}"
            )

           

        
    except KeyboardInterrupt:
        pass
    mean_reward = np.mean(episode_rewards)
    std_reward = np.std(episode_rewards)
    mean_success, std_success = np.mean(episode_success), np.std(episode_success)

    mean_len, std_len = np.mean(episode_lengths), np.std(episode_lengths)
    print("\n-------------------------")
    print("    ==== Results ====")
    print(f"Episode_reward={mean_reward:.2f} +/- {std_reward:.2f}")
    print(f"Episode_length={mean_len:.2f} +/- {std_len:.2f}")
    print(f"Episode_success={mean_success:.2f} +/- {std_success:.2f}")
    print("-------------------------")
        
    # Close process
    enjoy_env.close()