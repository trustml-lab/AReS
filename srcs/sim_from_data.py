import os, sys
import argparse
import numpy as np
import copy
import warnings
import pickle
import time
import threading
import torch as tc

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.abspath(os.path.join(current_dir, '..'))
sys.path.append(parent_dir)

import utils
import acon2
import ares


# Drone simulator
class Drone1D:
    def __init__(self, args):
        self.state = np.zeros((3, 1))  # Initial state: [rollspeed, pitchspeed, yawspeed]
        self.u = np.array([[1500], [1500], [1500], [1500]])  # Default servo values
        
        self.name = ['rollspeed', 'pitchspeed', 'yawspeed']
        self.dyn_mdl = np.array([[1, 0, 0], [1, 1, 0], [0, 1, 1]])  # Simple dynamics matrix
        self.obs_noise_sig = np.array([0.01, 0.01, 0.01])  # Observation noise

    def update(self):
        # Update the state based on dynamics and control input
        self.state = np.matmul(self.dyn_mdl, self.state) + self.u[:3] * 1e-4

    def observe(self):
        # Add noise to the state to simulate sensor observations
        sensors = {
            'rollspeed': np.random.normal(self.state[0, 0], self.obs_noise_sig[0]),
            'pitchspeed': np.random.normal(self.state[1, 0], self.obs_noise_sig[1]),
            'yawspeed': np.random.normal(self.state[2, 0], self.obs_noise_sig[2]),
            'GPS': self.state  # Added 'GPS' key
        }
        return {'o': sensors, 'u': self.u}


# Modified loop function
def loop(args):
    # Initialize the Drone1D simulator
    drone = Drone1D(args)
    ares = AReS(args)

    # Initialize baseline control invariant
    args_baseline = copy.deepcopy(args)
    args_baseline.exp_name = 'baseline'
    ci = ControlInv(args_baseline)

    # Initialize variables
    x_cur = None
    u_cur = np.zeros((4, 1))  # Default servo values to avoid NoneType error

    while True:
        # Update the drone state
        drone.update()
        
        # Simulate sensor observations
        observation = drone.observe()
        sensors = observation['o']
        u_cur = observation['u'] if observation['u'] is not None else u_cur  # Ensure u_cur is not None
        
        # Construct state vector from sensors
        x = np.array([[sensors['rollspeed']], [sensors['pitchspeed']], [sensors['yawspeed']]])
        
        if x_cur is not None and u_cur is not None:
            # Update AReS and ControlInv with observations
            obs = {'o': sensors, 'u': u_cur}
            ares.update(obs)
            ci.update(obs)

            # Reset x_cur for the next iteration
            x_cur = None
            time.sleep(0.05)
        
        # Update x_cur for the next iteration
        x_cur = x

        # Print simulated data for debugging
        print(f'[angular velocity] rollspeed = {x[0][0]:.4f}, pitchspeed = {x[1][0]:.4f}, yawspeed = {x[2][0]:.4f}')
        print(f'[servo data] servo_1 = {u_cur[0][0]}, servo_2 = {u_cur[1][0]}, servo_3 = {u_cur[2][0]}, servo_4 = {u_cur[3][0]}')


            

# adaptive reachable sets
class AReS:
    def __init__(self, args):
        self.args = args
        self.model = ares.models.LinearDynamicalSystem(args.model_base)
        self.learner = ares.learning.OnlineSystemLearning(args.model_base, self.model)
        
        self.obs = None
        self.data_log = []

        
    def __del__(self):
        data_fn = os.path.join(self.args.output_root, self.args.exp_name, 'data.pk')
        os.makedirs(os.path.dirname(data_fn), exist_ok=True)
        pickle.dump(self.data_log, open(data_fn, 'wb'))

                                        
    def update(self, obs):
        if self.obs is not None:
            try:
                x = self.obs['o']['GPS']  # Ensure 'GPS' key exists
            except KeyError:
                raise KeyError("'GPS' key is missing in the observation data")

            u = self.obs['u']
            x_next = obs['o']['GPS']
            self.learner.update(x, u, x_next)
            
            print(f'[{self.args.exp_name}]', self.learner.data)
            self.data_log.append(self.learner.data)
        self.obs = obs


        
class ControlInv(AReS):
    def __init__(self, args):
        super().__init__(args)
        
        self.model = ares.models.LinearDynamicalSystem(args.model_base)
        warnings.warn('TODO')
        self.model.load_state_dict(tc.load(os.path.join(args.output_root, args.exp_name, 'model')))

        self.learner = ares.learning.BatchSystemLearning(args.model_base, self.model)


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
    parser.add_argument('--model_base.lr', type=float, default=1e2)
    parser.add_argument('--model_base.state_dim', type=int, default=3)
    parser.add_argument('--model_base.action_dim', type=int, default=4)


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
    

