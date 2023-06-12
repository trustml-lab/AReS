import os, sys
import numpy as np

import torch as tc
from torch import nn, optim

class OnlineSystemLearning(nn.Module):
    def __init__(self, args, model):
        super().__init__()
        self.args = args
        self.model = model
        self.opt = optim.SGD(self.model.parameters(), lr=self.args.lr)
        #TODO: mse by default
        self.loss_fn = lambda x1, x2: (x1 - x2).pow(2).mean()

            
    def update(self, x, u, x_next):
        if type(x) is not tc.Tensor:
            x = tc.tensor(x, dtype=tc.float32)
        if type(u) is not tc.Tensor:
            u = tc.tensor(u, dtype=tc.float32)
        if type(x_next) is not tc.Tensor:
            x_next = tc.tensor(x_next, dtype=tc.float32)

        self.opt.zero_grad()

        print(self.model(x, u) - x_next)
        loss = self.loss_fn(self.model(x, u), x_next)
        
        
        
        loss.backward()
        self.opt.step()

        print(f'loss = {loss.item()}')

