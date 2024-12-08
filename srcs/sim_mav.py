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

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

POSCTL_TAKEOFF_THROTTLE_AMOUNT = 1000
POSCTL_FLOAT_THROTTLE_AMOUNT = 500


# # drone simulator
# class Drone1D:
#     def __init__(self, args):
#         self.state = np.zeros((3, 1))
#         self.u = np.array([[1e-5], [0], [0]])

#         self.name = args.name
#         self.dyn_mdl = np.array([[1, 0, 0], [1, 1, 0], [0, 1, 1]])
#         self.obs_noise_sig = np.array([0.001, 0.005, 0.01])
#         #TODO: add a PID controller

#     def update(self):
#         #TODO: update self.u
#         self.state = np.matmul(self.dyn_mdl, self.state) + self.u


#     def observe(self):
#         sensors = {n: np.random.normal(self.state, sig) for n, sig in zip(self.name, self.obs_noise_sig)}
#         return {'o': sensors, 'u': self.u}
            

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
            x = self.obs['o']['GPS'] ##TODO: state estimation
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
        
        

def loop(args):

    # init mav
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

    # make sure the connection is valid
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
            (master.target_system, master.target_component))

    #drone = Drone1D(args.data)
    
    ares = AReS(args)

    #TODO
    args_baseline = copy.deepcopy(args)
    args_baseline.exp_name = 'baseline'
    ci = ControlInv(args_baseline)
    
    # loops
    x_cur = None
    u_cur = None
    while True:
        msg = master.recv_match()
        if not msg:
            continue
        #print(msg)

        if msg.get_type() == 'ATTITUDE':
            msg = msg.to_dict()
            ms = msg["time_boot_ms"]
            rv = msg["rollspeed"]
            pv = msg["pitchspeed"]
            yv = msg["yawspeed"]
            x = np.array([[rv], [pv], [yv]])
            
            if x_cur is not None and u_cur is not None:
                # update
                sensors = {'GPS': x_cur}
                obs = {'o': sensors, 'u': u_cur}
                ares.update(obs)
                ci.update(obs)

                # init
                x_cur = None
                u_cur = None
                time.sleep(0.05)
                
            # update
            x_cur = x
            
            #print(f'[angular velocity, {ms} ms] rollspeed = {rv:.4f}, pitchspeed = {pv:.4f}, yawspeed = {yv:.4f}')
            
        elif msg.get_type() == 'SERVO_OUTPUT_RAW':
            msg = msg.to_dict()
            ms = msg["time_usec"]//1000
            s1 = msg["servo1_raw"]
            s2 = msg["servo2_raw"]
            s3 = msg["servo3_raw"]
            s4 = msg["servo4_raw"]
            u_cur = np.array([[s1], [s2], [s3], [s4]])
            #print(f'[servo data, {ms} ms] servo_1 = {s1}, servo_2 = {s2}, servo_3 = {s3}, servo_4 = {s4}')


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
    

