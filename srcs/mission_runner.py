import time
import sys
import os

# force MAVLink 2.0
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil

POSCTL_TAKEOFF_THROTTLE_AMOUNT = 1000
POSCTL_FLOAT_THROTTLE_AMOUNT = 500

def connect_mavlink(sim=False):
    if sim:
        master = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    else:
        known_devices=[
            '*FTDI*',
            "*Arduino_Mega_2560*",
            "*3D_Robotics*",
            "*USB_to_UART*",
            '*PX4*',
            '*FMU*',
            "*Gumstix*"
        ]
        serial_list = mavutil.auto_detect_serial(preferred_list=known_devices)

        if len(serial_list) == 0:
            print("Error: no serial connection found")
            exit(1)

        if len(serial_list) > 1:
            print('Auto-detected serial ports are:')
            for port in serial_list:
                print(" {:}".format(port))
        print('Using port {:}'.format(serial_list[0]))
        port = serial_list[0].device

        master = mavutil.mavlink_connection(port, baud=57600, source_system=255)

    # make sure the connection is valid
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
            (master.target_system, master.target_component))

    return master


def connect_virt_rc(master):
    set_param(
        master,
        "COM_RC_LOSS_T",
        30,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )

    master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        750,
        0,
        0
    )


def lower_virt_rc(master):
    master.mav.manual_control_send(
        master.target_system,
        0,
        0,
        0,
        0,
        0
    )


def set_param(master, param_name, param_value, param_type, debug=False):
    print(f"[*] (MAVLink) set param {param_name} := {param_value}")
    current_param_name = param_name
    current_param_type = param_type

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode(), # requires 'b'
        param_value,
        param_type,
    )

    # Verify parameter change
    master.mav.param_request_list_send(
        master.target_system,
        master.target_component
    )

    while True:
        time.sleep(0.01)
        try:
            message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()

            if message["param_id"] == param_name:
                print('name: %s\tvalue: %d' % (message['param_id'],
                                               message['param_value']))
                break

        except Exception as error:
            print(error)
            sys.exit(0)


def arm(master):
    # set / connect (virtual) RC before arming to prevent px4 from
    # engaging the failsafe mode right away
    print("[send] Arm")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, # 1: arm, 0: disarm
        21196,
        0, 0, 0, 0, 0
    )
    master.motors_armed_wait()
    print("armed!")
    time.sleep(2)


def disarm(master, force=False):
    print("[send] Disarm")
    if force:
        param_force = 21196
    else:
        param_force = 0

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, # 1: arm, 0: disarm
        param_force,
        0, 0, 0, 0, 0
    )
    master.motors_disarmed_wait()
    print("disarmed!")
    time.sleep(2)


def get_mode(master):
    while True:
        msg = master.recv_match(type = 'HEARTBEAT', blocking = False)
        if msg:
            mode = mavutil.mode_string_v10(msg)
            print(mode)
            break


def set_mode(master, mode_str, wait_time):
    """
    mode_str can be one of:
    {
        'MANUAL': (81, 1, 0),
        'STABILIZED': (81, 7, 0),
        'ACRO': (65, 5, 0),
        'RATTITUDE': (65, 8, 0),
        'ALTCTL': (81, 2, 0),
        'POSCTL': (81, 3, 0),
        'LOITER': (29, 4, 3),
        'MISSION': (29, 4, 4),
        'RTL': (29, 4, 5),
        'LAND': (29, 4, 6),
        'RTGS': (29, 4, 7),
        'FOLLOWME': (29, 4, 8),
        'OFFBOARD': (29, 6, 0),
        'TAKEOFF': (29, 4, 2)
    }
    """
    print(f"[+] Requesting mode switch: {mode_str}")
    (mode, custom_mode, custom_sub_mode) = master.mode_mapping()[mode_str]

    while True:
        # set mode
        master.set_mode(mode, custom_mode, custom_sub_mode)

        # wait for ACK command
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=False)

        if ack_msg is None:
            continue

        ack_msg = ack_msg.to_dict()
        print("  [*] mode ack:", ack_msg)

        # Check if command in the same in `set_mode`
        if ack_msg['command'] != mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            continue

        if ack_msg['result'] != 0:
            print("  [*] retry")
            time.sleep(0.5)
            continue

        print(  "[*] result:", mavutil.mavlink.enums['MAV_RESULT'][ack_msg['result']].description)
        break

    print(f"[+] mode set to {mode_str}")

    time.sleep(wait_time)


def prep_mission(master):
    print("[+] Getting prepared for a mission")

    print("  [*] Take off")
    for i in range(50):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0, # y
            POSCTL_TAKEOFF_THROTTLE_AMOUNT, # z
            0, # r
            0)
        time.sleep(0.1)

    print("  [*] Float")
    for i in range(50):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0, # y
            POSCTL_FLOAT_THROTTLE_AMOUNT, # z
            0, # r
            0)
        time.sleep(0.1)

def do_mission(master, is_done, mission):
    print("Starting a mission")
    cnt = 0
    for (x, y, z, sec) in mission:
        print(f"Target #{cnt}: ({x}, {y}, {z}) for {sec} sec")

        for i in range(sec * 10):
            master.mav.manual_control_send(
                master.target_system,
                x, # x
                y, # y
                z, # z
                0, # r
                0)
            time.sleep(0.1) # 100 hz

        cnt += 1

    is_done.set()
    print("Mission complete")


def end_mission(master):
    print("[+] Ending a mission")
    print("  [*] Landing")

    for i in range(200):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0, # y
            0, # z
            0, # r
            0)
        time.sleep(0.1)


if __name__ == "__main__":
    set_mode(master, "POSCTL", wait_time=1)
    print("takeoff")

    print("go to target")
    for i in range(500):
        master.mav.manual_control_send(
            master.target_system,
            500, # x
            500, # y
            POSCTL_FLOAT_THROTTLE_AMOUNT, # z
            0, # r
            0)
        time.sleep(0.01)

    print("float")
    for i in range(500):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0, # y
            POSCTL_FLOAT_THROTTLE_AMOUNT, # z
            0, # r
            0)
        time.sleep(0.01)

    print("go back home")
    for i in range(500):
        master.mav.manual_control_send(
            master.target_system,
            -500, # x
            -500,# y
            POSCTL_FLOAT_THROTTLE_AMOUNT, # z
            0, # r
            0)
        time.sleep(0.01)

    print("float")
    for i in range(500):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0, # y
            POSCTL_FLOAT_THROTTLE_AMOUNT, # z
            0, # r
            0)
        time.sleep(0.01)

    print("landing")
    for i in range(500):
        master.mav.manual_control_send(
            master.target_system,
            0, # x
            0, # y
            0, # z
            0, # r
            0)
        time.sleep(0.01)

