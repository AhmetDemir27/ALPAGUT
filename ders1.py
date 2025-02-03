from pymavlink import mavutil


uav = mavutil.mavlink_connection('127.0.0.1:14550')


uav.wait_heartbeat()

print("baglandi")