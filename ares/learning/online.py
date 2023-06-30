import os, sys
import numpy as np
import warnings

import torch as tc
from torch import nn, optim


class OnlineSystemLearning(nn.Module):
    def __init__(self, args, model):
        super().__init__()
        self.args = args
        self.model = model
        self.opt = optim.SGD(self.model.parameters(), lr=self.args.lr)
        #TODO: mse by default
        #self.loss_fn = lambda x1, x2: (x1 - x2).pow(2).mean()
        self.loss_fn = lambda x1, x2: (x1 - x2).abs().mean(dim=-1)

        self.n_batch = 100
        self.x_batch = []
        self.u_batch = []
        self.y_batch = []
        self.data = {}
        
            
    def update(self, x, u, x_next):
        if type(x) is not tc.Tensor:
            x = tc.tensor(x, dtype=tc.float32)
        if type(u) is not tc.Tensor:
            u = tc.tensor(u, dtype=tc.float32)
        if type(x_next) is not tc.Tensor:
            x_next = tc.tensor(x_next, dtype=tc.float32)


        self.x_batch.append(x.t())
        self.u_batch.append(u.t())
        self.y_batch.append(x_next.t())
        self.x_batch = self.x_batch[-self.n_batch:]
        self.u_batch = self.u_batch[-self.n_batch:]
        self.y_batch = self.y_batch[-self.n_batch:]
            
        # self.opt.zero_grad()

        # x = tc.cat(self.x_batch)
        # u = tc.cat(self.u_batch)
        # y = tc.cat(self.y_batch)
        
        # print((self.model(x, u) - y).mean(dim=0))
        # loss = self.loss_fn(self.model(x, u), y).mean()
        
        # loss.backward()
        # self.opt.step()

        x = tc.cat(self.x_batch).numpy()
        u = tc.cat(self.u_batch).numpy()
        y = tc.cat(self.y_batch).numpy()

        AB = np.matmul(y.transpose(), np.linalg.pinv(np.hstack((x, u)).transpose()))
        A = AB[:, :3]
        B = AB[:, 3:]

        self.model.A.data = tc.tensor(A)
        self.model.B.data = tc.tensor(B)
        
        
        #print('A =', self.model.A)
        
        y_pred = self.model(tc.tensor(x), tc.tensor(u))
        loss = self.loss_fn(y_pred, tc.tensor(y))[-1]

        self.data = {'x': x[-1], 'u': u[-1], 'y': y[-1], 'y_pred': y_pred[-1].detach().numpy(), 'loss': loss.item()}

        #print(f'loss = {loss.item()}')

