import time
import sys
import os
import pickle
import argparse
import threading

import mission_runner as mr

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

POSCTL_TAKEOFF_THROTTLE_AMOUNT = 700
POSCTL_FLOAT_THROTTLE_AMOUNT = 500


def request_message_interval(master, message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz

    Code borrowed from: https://www.ardusub.com/developers/pymavlink.html
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0
    )


def collect_data(master, ang_vel, servo, is_done, n_data_max=200):
    print("collecting data")

    n_data = 0
    #TODO: trigger the start and end via messages.

    while n_data <= n_data_max and not is_done.wait(0):
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

            # print(f'[angular velocity, {ms} ms] rollspeed = {rv:.4f}, pitchspeed = {pv:.4f}, yawspeed = {yv:.4f}')

        elif msg.get_type() == 'SERVO_OUTPUT_RAW':
            msg = msg.to_dict()
            ms = msg["time_usec"]//1000
            s1 = msg["servo1_raw"]
            s2 = msg["servo2_raw"]
            s3 = msg["servo3_raw"]
            s4 = msg["servo4_raw"]
            servo[ms//100] = {"servo1": s1, "servo2": s2, "servo3": s3, "servo4": s4}
            # print(f'[servo data, {ms} ms] servo_1 = {s1}, servo_2 = {s2}, servo_3 = {s3}, servo_4 = {s4}')

    print("done collecting data")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='data collection')
    parser.add_argument('--output_root', type=str, default='output_data_collection')
    parser.add_argument('--exp_name', type=str, default='1')
    parser.add_argument('--sim', type=bool, default=False)
    args = parser.parse_args()

    exp_output_root = os.path.join(args.output_root, args.exp_name)
    os.makedirs(exp_output_root, exist_ok=True)

    master = mr.connect_mavlink(sim=args.sim)
    mr.connect_virt_rc(master)

    time.sleep(1)
    mr.lower_virt_rc(master)
    time.sleep(1)

    # set msg interval to 20 hz
    request_message_interval(master, 36, 10) # SERVO_OUTPUT_RAW
    request_message_interval(master, 141, 10) # ATTITUDE

    time.sleep(1)
    mr.set_mode(master, "POSCTL", wait_time=1)

    time.sleep(1)

    mr.arm(master)
    time.sleep(1)

    mr.prep_mission(master)

    rc_mission = [
        (500, 500, POSCTL_FLOAT_THROTTLE_AMOUNT, 3),
        (0, 0, POSCTL_FLOAT_THROTTLE_AMOUNT, 5),
        (-500, -500, POSCTL_FLOAT_THROTTLE_AMOUNT, 3),
        (0, 0, POSCTL_FLOAT_THROTTLE_AMOUNT, 5),
    ]
    is_done = threading.Event()

    t_mission = threading.Thread(
        target=mr.do_mission,
        args=(master,is_done,rc_mission,)
    )
    # mr.do_mission(master, rc_mission)

    n_data_max = 2000
    ang_vel = dict()
    servo = dict()
    t_data = threading.Thread(
        target=collect_data,
        args=(master,ang_vel,servo,is_done,n_data_max,)
    )
    # collect_data(master, ang_vel, servo, n_data_max=20)

    threads = [t_data, t_mission]
    for t in threads:
        t.start()

    for t in threads:
        t.join()

    print("Threads done")
    mr.end_mission(master)
    mr.disarm(master, force=True)
    mr.lower_virt_rc(master)

    ##TODO: how to synchronize data?

    # note: dumping raw ang_vel and servo in case sync'ing doesn't work as
    # expected.

    # dump raw ang_vel data
    with open(os.path.join(exp_output_root, "data_ang_vel.pk"), "wb") as f:
        pickle.dump(ang_vel, f)

    # dump raw servo data
    with open(os.path.join(exp_output_root, "data_servo.pk"), "wb") as f:
        pickle.dump(ang_vel, f)

    # match data by time
    data = []
    for k, v in ang_vel.items():
        if k in servo:
            data.append({'timestamp': k, **ang_vel[k], **servo[k]})
    pickle.dump(data, open(os.path.join(exp_output_root, "data.pk"), "wb"))

