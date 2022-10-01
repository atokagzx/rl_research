import torch as th

reach = {
    "PPO": dict(
        batch_size=8,
        gamma=0.95,
        n_steps=256,
        learning_rate=8.48454676231986e-05,
        ent_coef=4.4997033264033236e-08,
        clip_range=0.4,
        n_epochs=10,
        gae_lambda=0.99,
        max_grad_norm=2,
        vf_coef=0.3531062405315326,
        policy_kwargs=dict(
            activation_fn=th.nn.ReLU,
            net_arch=[dict(pi=[32, 32],
                           vf=[32, 32])]),
    )
}