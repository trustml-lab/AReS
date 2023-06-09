import os, sys
import torch as tc
from torch import nn

class LinearDynamicalSystem(nn.Module):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.A = nn.Parameter(tc.zeros((args.state_dim, args.state_dim)))
        self.B = nn.Parameter(tc.zeros((args.action_dim, args.action_dim)))


    def forward(self, x, u):
        x_next = tc.mm(self.A, x) + tc.mm(self.B, u)
        return x_next
