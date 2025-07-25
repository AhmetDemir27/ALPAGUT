from pymavlink import mavutil
import sys

master = mavutil.mavlink_connection('127.0.0.1:14550')
master.wait_heartbeat()

print("baglandi")

mode = 'TAKEOFF'

#degistirecek olan modun uygun olup olmadigini kontrol ediyor.
if mode not in master.mode_mapping():
    print('Unknown mode : {}'.format(mode))
    print('Try:', list(master.mode_mapping().keys()))
    sys.exit(1)

#mod ID sini aliyor

mode_id = master.mode_mapping()[mode]
#modu degistiren komut
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id)

master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

print("komut verildi")

master.motors_armed_wait()

print("arm gerceklesti")



# master.mav.command_long_send(
# master.target_system,
# master.target_component,
# mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#      0,
#      0, 0, 0, 0, 0, 0, 25)


master.mav.command_long_send(
     master.target_system,
     master.target_component,
     mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
     0,
     0, 0, 0, 0, 0, 0, 0)


while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'HEARTBEAT':
        # print("\n\n*****Got message: %s*****" % msg.get_type())
        # print("Message: %s" % msg)
        # print("\nAs dictionary: %s" % msg.to_dict())
        # # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
        # print("\nSystem status: %s" % msg.system_status)
        # GLOBAL_POSITION_INT mesajını bekler ve alır
        msg1 = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)

        # Mesajın içeriğini yazdırır
        print(msg1)

        # Yükseklik, X ve Y değerlerini hesaplar
        altitude = msg1.relative_alt / 1000  # Yükseklik (metre)
        x_position = msg1.lat / 1e7          # Enlem (derece)
        y_position = msg1.lon / 1e7          # Boylam (derece)

        # Değerleri yazdırır
        print(f"Altitude: {altitude} m")
        print(f"X (Latitude): {x_position}°")
        print(f"Y (Longitude): {y_position}°")