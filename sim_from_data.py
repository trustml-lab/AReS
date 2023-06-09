import os, sys
import argparse
import numpy as np
import copy
import warnings
import pickle
import time

import utils
import acon2
import ares


# drone simulator
class Drone1D:
    def __init__(self, args):
        self.state = np.zeros((3, 1))
        self.u = np.array([[1e-5], [0], [0]])

        self.name = args.name
        self.dyn_mdl = np.array([[1, 0, 0], [1, 1, 0], [0, 1, 1]])
        self.obs_noise_sig = np.array([0.001, 0.005, 0.01])
        #TODO: add a PID controller

    def update(self):
        #TODO: update self.u
        self.state = np.matmul(self.dyn_mdl, self.state) + self.u


    def observe(self):
        sensors = {n: np.random.normal(self.state, sig) for n, sig in zip(self.name, self.obs_noise_sig)}
        return {'o': sensors, 'u': self.u}
            

# adaptive reachable sets
class AReS:
    def __init__(self, args):

        # # init acon2
        # model_base = {k: getattr(acon2, v)(model_base_args) for k, v, model_base_args in zip(args.data.name, args.model_base.name, split_args(args.model_base))}
        # model_ps_src = {k: getattr(acon2, model_name)(model_args, model_base[k]) for k, model_name, model_args in zip(args.data.name, args.model_ps.name, split_args(args.model_ps))}        
        # self.acon2 = acon2.ACon2(args.model_ps, model_ps_src)

        self.model = ares.models.LinearDynamicalSystem(args.model_base)
        self.learner = ares.learning.OnlineSystemLearning(args.model_base, self.model)
        
        self.obs = None

        
    def update(self, obs):

        if self.obs is not None:
            x = self.obs['o']['GPS'] ##TODO: state estimation
            u = self.obs['u']
            x_next = obs['o']['GPS']
            self.learner.update(x, u, x_next)
            
        self.obs = obs
        
        # if not self.acon2.initialized:
        #     self.acon2.init_or_update(obs)
        # else:
        #     self.acon2.init_or_update(obs)
        #     print(f"[time = {self.counter}] median(obs) = {np.median([obs[k] for k in obs.keys() if obs[k] is not None]):.4f}, "\
        #           f"interval = [{self.acon2.ps[0]:.4f}, {self.acon2.ps[1]:.4f}], length = {self.acon2.ps[1] - self.acon2.ps[0]:.4f}, "\
        #           f"error = {self.acon2.n_err / self.acon2.n_obs:.4f}")


def loop(args):
    drone = Drone1D(args.data)
    ares = AReS(args)
    for t in range(1, 101):
        drone.update()
        obs = drone.observe()
        ares.update(obs)
        time.sleep(0.1)
    

# others            
def split_args(args):
    args_split = []
    for i in range(len(args.name)):
        args_new = copy.deepcopy(args)
        for d in args.__dict__:
            if type(getattr(args, d)) == list:
                setattr(args_new, d, getattr(args, d)[i])
        args_split.append(args_new)
    return args_split



def parse_args():
    ## init a parser
    parser = argparse.ArgumentParser(description='adaptive learning')

    ## meta args
    parser.add_argument('--exp_name', type=str, required=True)
    parser.add_argument('--output_root', type=str, default='output')

    # ## data args
    # parser.add_argument('--data.name', type=str, default='PriceDataset')
    # parser.add_argument('--data.path', type=str, nargs='+', default=[
    #     'data/price_USD_ETH/coinbase',
    #     'data/price_USD_ETH/binance',
    #     'data/price_USD_ETH/UniswapV2',
    # ])
    # parser.add_argument('--data.start_time', type=str, default='2021-01-01T00:00')
    # parser.add_argument('--data.end_time', type=str, default='2021-12-31T23:59')
    # parser.add_argument('--data.time_step_sec', type=int, default=60) #60
    # parser.add_argument('--data.seed', type=lambda v: None if v=='None' else int(v), default=0)

    ## data args
    parser.add_argument('--data.name', type=str, nargs='+', default=['GPS', 'IMU', 'Camera'])

    
    ## model args
    #parser.add_argument('--model_base.name', type=str, nargs='+', default=['KF1D'])    
    parser.add_argument('--model_base.lr', type=float, default=1e-1)
    parser.add_argument('--model_base.state_dim', type=int, default=3)
    parser.add_argument('--model_base.action_dim', type=int, default=3)


    # ## model args
    # parser.add_argument('--model_base.name', type=str, nargs='+', default=['KF1D'])    
    # parser.add_argument('--model_base.lr', type=float, nargs='+', default=[1e-3])
    # parser.add_argument('--model_base.state_noise_init', type=float, nargs='+', default=[0.1])
    # parser.add_argument('--model_base.obs_noise_init', type=float, nargs='+', default=[0.1])
    # parser.add_argument('--model_base.score_max', type=float, nargs='+', default=[1])

    # parser.add_argument('--model_ps.name', type=str, nargs='+', default=['SpecialMVP'])    
    # parser.add_argument('--model_ps.n_bins', type=int, nargs='+', default=[100])

    # parser.add_argument('--model_ps.K', type=int, default=3)
    # parser.add_argument('--model_ps.alpha', type=float, nargs='+', default=[0.01])
    # parser.add_argument('--model_ps.beta', type=int, default=1) 
    # parser.add_argument('--model_ps.eta', type=float, default=5)
    # parser.add_argument('--model_ps.nonconsensus_param', type=float, default=0) 
    
    args = parser.parse_args()
    args = utils.to_tree_namespace(args)
    args = utils.propagate_args(args, 'exp_name')
    args = utils.propagate_args(args, 'output_root')
    
    # # duplicate options
    # assert(args.model_ps.K == len(args.data.name))
    # if args.model_ps.K >= 2:
    #     if len(args.model_base.name) == 1:
    #         args.model_base.name = args.model_base.name*args.model_ps.K
    #     if len(args.model_base.lr) == 1:
    #         args.model_base.lr = args.model_base.lr*args.model_ps.K
    #     if len(args.model_base.state_noise_init) == 1:
    #         args.model_base.state_noise_init = args.model_base.state_noise_init*args.model_ps.K
    #     if len(args.model_base.obs_noise_init) == 1:
    #         args.model_base.obs_noise_init = args.model_base.obs_noise_init*args.model_ps.K
    #     if len(args.model_base.score_max) == 1:
    #         args.model_base.score_max = args.model_base.score_max*args.model_ps.K
    #     if len(args.model_ps.name) == 1:
    #         args.model_ps.name = args.model_ps.name*args.model_ps.K
    #     if len(args.model_ps.n_bins) == 1:
    #         args.model_ps.n_bins = args.model_ps.n_bins*args.model_ps.K
    #     if len(args.model_ps.alpha) == 1:
    #         args.model_ps.alpha = args.model_ps.alpha*args.model_ps.K
    
    
    ## set loggers
    os.makedirs(os.path.join(args.output_root, args.exp_name), exist_ok=True)
    sys.stdout = utils.Logger(os.path.join(args.output_root, args.exp_name, 'out'))
    
    ## print args
    utils.print_args(args)
    
    return args


if __name__ == '__main__':
    args = parse_args()
    loop(args)
    
    
