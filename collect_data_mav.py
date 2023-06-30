import time
import sys
import os
import pickle
import argparse

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

POSCTL_TAKEOFF_THROTTLE_AMOUNT = 1000
POSCTL_FLOAT_THROTTLE_AMOUNT = 500


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='data collection')
    parser.add_argument('--output_root', type=str, default='output_data_collection')
    parser.add_argument('--exp_name', type=str, default='1')
    args = parser.parse_args()

    exp_output_root = os.path.join(args.output_root, args.exp_name)
    os.makedirs(exp_output_root, exist_ok=True)
    
    master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")

    # make sure the connection is valid
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
            (master.target_system, master.target_component))


    n_data = 0
    #TODO: trigger the start and end via messages.
    n_data_max = 1000
    ang_vel = {}
    servo = {}
    
    while n_data <= n_data_max:
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
            ang_vel[ms//100] = {"rollspeed": rv, "pitchspeed": pv, "yawspeed": yv} # collect data in 10 Hz
            n_data += 1
            
            print(f'[angular velocity, {ms} ms] rollspeed = {rv:.4f}, pitchspeed = {pv:.4f}, yawspeed = {yv:.4f}')
            
        elif msg.get_type() == 'SERVO_OUTPUT_RAW':
            msg = msg.to_dict()
            ms = msg["time_usec"]//1000
            s1 = msg["servo1_raw"]
            s2 = msg["servo2_raw"]
            s3 = msg["servo3_raw"]
            s4 = msg["servo4_raw"]
            servo[ms//100] = {"servo1": s1, "servo2": s2, "servo3": s3, "servo4": s4}
            print(f'[servo data, {ms} ms] servo_1 = {s1}, servo_2 = {s2}, servo_3 = {s3}, servo_4 = {s4}')

    ##TODO: how to synchronize data?
    # match data by time
    data = []
    for k, v in ang_vel.items():
        if k in servo:
            data.append({'timestamp': k, **ang_vel[k], **servo[k]})
    pickle.dump(data, open(os.path.join(exp_output_root, "data.pk"), "wb"))
    
    