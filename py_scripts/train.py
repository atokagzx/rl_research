# -*- coding: utf-8 -*-
#!/usr/bin/python3

import argparse
import numpy as np
from stable_baselines3 import SAC, TD3, PPO
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
import torch as th

# Instantiate the parser
parser = argparse.ArgumentParser(description='Trainer optional arguments')

parser.add_argument('--headless',
                    action='store_true',
                    help="do not use bullet GUI", 
                    default=False)

parser.add_argument('-t', '--tsteps', 
                    type=float, 
                    default=1e6,
                    help='number of timesteps (default: 1e6)')       

parser.add_argument('-a', '--algo',
                    type=str, 
                    default="PPO",
                    help='[PPO] (default: PPO)')

parser.add_argument('-s', '--skill', 
                    type=str, 
                    default='reach',
                    help='[reach] (default: reach)') 

if __name__ == "__main__":

    args = parser.parse_args()
    print("-----------------------------------------\n")
    print("skill  :", args.skill)
    print("algo   :", args.algo)
    print("viz    :", not args.headless)
    print("tsteps :", args.tsteps)
    print("\n-----------------------------------------")

    RL_ALGORITHM = args.algo
    N_TIMESTEPS = int(args.tsteps)

    #choose environment for skill 
    try:
        if args.skill == 'reach': 
            from envs.reach_env import Env as ur_env
            from hyperparams import reach as hyperparams
        else:
            print('ERROR: Requested skill "{0}" not implemented yet.'.format(args.skill)) 
    except ImportError as error:
        print('ERROR: Requested skill "{0}" not implemented yet.'.format(args.skill))
        exit()

    if args is not None:
        headless = args.headless

    save_path = "weights/UR_ENV_" + args.skill
    tensorboard_path = "tensorboard_log/" + args.skill + "/"

    # Create the evaluation environment and callbacks
    train_env = Monitor(ur_env(headless))

    callbacks = [EvalCallback(train_env, best_model_save_path=save_path)]

    # Save a checkpoint every n steps
    callbacks.append(
        CheckpointCallback(
            save_freq=100000, save_path=save_path, name_prefix="rl_model"
        )
    )

    algo = {
        "PPO": PPO
    }[RL_ALGORITHM]

    n_actions = ur_env.action_space.shape[0]

    # Tuned hyperparameters from https://github.com/DLR-RM/rl-baselines3-zoo
    
    train_env.reset()
    model = algo('MultiInputPolicy', train_env, verbose=1, tensorboard_log=tensorboard_path, **hyperparams[RL_ALGORITHM])

    try:
        model.learn(N_TIMESTEPS, callback=callbacks)
    except KeyboardInterrupt:
        pass
    print(f"Saving to {save_path}.zip")
    model.save(save_path)

    train_env.close()