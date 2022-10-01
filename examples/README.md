# Table of content
-  [Training](#train)
-  [Evaluation](#eval)
## Train your agent <a name="train"></a>
In Docker-container go to the required skill directory.\
To start agent train run command:
```bash
python3 train.py --skill reach --algo PPO --tsteps 10e6 --headless
```
### Tensorboard <a name="eval"></a>
TensorBoard provides the visualization for training, use it with:
```bash
tensorboard --logdir tensorboard_log
```
## Evaluate trained agent <a name="eval"></a>
Run enjoy script when training done:
```bash
python3 enjoy.py --skill reach --algo PPO -m models/reach/UR_ENV_reach/best_model.zip --screencast
```
**NOTE:** By default enjoy.py loads model from ```rl_skills/trained_models/```
