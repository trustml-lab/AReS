import os, sys
import torch as tc
from torch import nn

class LinearDynamicalSystem(nn.Module):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.A = nn.Parameter(tc.zeros((args.state_dim, args.state_dim)))
        self.B = nn.Parameter(tc.zeros((args.state_dim, args.action_dim)))

        for p in self.parameters():
            nn.init.normal_(p)


    def forward(self, x, u):
        #print(x.shape, u.shape)
        x_next = tc.matmul(x, self.A.t()) + tc.matmul(u, self.B.t())
        #print(x_next.shape)
        return x_next
