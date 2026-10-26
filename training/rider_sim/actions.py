"""Learn common wheel drive and a smaller differential steering action.

The final linear actor head is algebraically folded back to native wheel signs.
Large latent bounds never clip a bounded tanh MLP's output; native wheels clip
after the linear transform, exactly as the deployed dense actor does.
"""
import gymnasium as gym
from gymnasium import spaces
import numpy as np


def transform_action(action,scale):
    action=np.asarray(action,np.float32)
    return np.clip(np.stack([action[...,0]-scale*action[...,1],-action[...,0]-scale*action[...,1]],axis=-1),-1,1).astype(np.float32)


class CommonTurnEnv(gym.ActionWrapper):
    def __init__(self,env,scale=.15):
        super().__init__(env);self.scale=scale
        self.action_space=spaces.Box(-1000,1000,(2,),np.float32)
    def action(self,action):return transform_action(action,self.scale)


def predict_native(model,observation):
    action=model.predict(observation,deterministic=True)[0]
    scale=getattr(model,'rider_common_turn_scale',None)
    return transform_action(action,scale) if scale is not None else action
