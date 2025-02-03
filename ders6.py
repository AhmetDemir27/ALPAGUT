from pymavlink import mavutil
import sys

master = mavutil.mavlink_connection('127.0.0.1:14550')
master.wait_heartbeat()

print("baglandi")

lat = -35.36311138
len = 149.16527338
alt = 14
mode = 'GUİDED'

master.mav.set_position_target_global_int_send(
    0, master.target_system,master.target_component,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0b110111111000,
    int(lat*12 ,1e7),
    int(len * 1e7),
    alt,
    0,0,0,0,0,0,0,0
    )