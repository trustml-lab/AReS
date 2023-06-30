import os, sys
import numpy as np
import warnings

import torch as tc
from torch import nn, optim

class BatchSystemLearning(nn.Module):
    def __init__(self, args, model):
        super().__init__()
        self.args = args
        self.model = model
        
        self.loss_fn = lambda x1, x2: (x1 - x2).abs().mean(dim=-1)
        self.data = {}
        
            
    def update(self, x, u, x_next):
        if type(x) is not tc.Tensor:
            x = tc.tensor(x, dtype=tc.float32)
        if type(u) is not tc.Tensor:
            u = tc.tensor(u, dtype=tc.float32)
        if type(x_next) is not tc.Tensor:
            x_next = tc.tensor(x_next, dtype=tc.float32)

        x = x.t()
        u = u.t()
        x_next = x_next.t()
        
        #print('A =', self.model.A)

        y = x_next
        y_pred = self.model(x, u)
        
        loss = self.loss_fn(y_pred, y)

        self.data = {'x': x[-1], 'u': u[-1], 'y': y[-1], 'y_pred': y_pred[-1].detach().numpy(), 'loss': loss.item()}

        #print(self.data)
        #print(f'loss = {loss.item()}')
