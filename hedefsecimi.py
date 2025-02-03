from pymavlink import mavutil
import sys
import math
import threading

def haversine(lat1, lon1, lat2, lon2):
    R = 6371e3  # Dünya'nın yarıçapı (metre)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # İki nokta arasındaki mesafe (metre)

class Ucak:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print(f"{connection_str} ile bağlandı")
        self.running = False
        self.position = {'lat': None, 'lon': None, 'alt': None, 'speed': None}
        self.konum_thread = None

    def mod_degistir(self,mode):
        if mode not in self.master.mode_mapping():
            print("Bilinmeyen Mod")
            print('Mevcut modlar:', list(self.master.mode_mapping().keys()))
            return
        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Mod f{mode} olarak değiştirildi")

    def motor_arm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("Komut verildi: Motor arm")
        self.master.motors_armed_wait()
        print("Motor arm işlemi gerçekleşti")

    def motor_disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_componenet,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0 , 0, 0, 0, 0, 0, 0
        )
        print("Motor disarm işlemi gerçekleşti")

    def inis_yap(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND_LOCAL,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("İniş komutu gönderildi")

    
    def konum_bilgisi_al(self):
        while self.running:
            msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
            if msg:
                self.position['lat'] = msg.lat / 1e7
                self.position['lon'] = msg.lon / 1e7
                self.position['alt'] = msg.relative_alt / 1000
                x_speed = msg.vx / 100
                y_speed = msg.vy / 100
                z_speed = msg.vz / 100
                self.position['speed'] = math.sqrt(x_speed**2 + y_speed**2 + z_speed**2)
    
    def start_konum_takip(self):
        if not self.running:
            self.running = True
            self.konum_thread = threading.Thread(target=self.konum_bilgisi_al, daemon=True)
            self.konum_thread.start()
    
    def durdur(self):
        self.running = False
        if self.konum_thread:
            self.konum_thread.join()

# Uçakları başlat
avci_ucak = Ucak('127.0.0.1:14550')
av1_ucak = Ucak('127.0.0.1:14560')
av2_ucak = Ucak('127.0.0.1:14570')
#av3_ucak = Ucak('127.0.0.1:14580')
#av4_ucak = Ucak('127.0.0.1:14590')


# Konum bilgisi takip başlat
avci_ucak.start_konum_takip()
av1_ucak.start_konum_takip()
av2_ucak.start_konum_takip()

# Anlık mesafe ölçümü
try:
    while True:
        if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], av1_ucak.position['lat'], av1_ucak.position['lon']):
            mesafe = haversine(
                avci_ucak.position['lat'], avci_ucak.position['lon'],
                av1_ucak.position['lat'], av1_ucak.position['lon']
            )
#            print(f"Av1_ucak ile arasındaki mesafe: {mesafe:.2f} metre\n")

        if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], av2_ucak.position['lat'], av2_ucak.position['lon']):
            mesafe2 = haversine(
                avci_ucak.position['lat'], avci_ucak.position['lon'],
                av2_ucak.position['lat'], av2_ucak.position['lon']
            )
#            print(f"Av1_ucak ile arasındaki mesafe: {mesafe2:.2f} metre\n")
        
        if(mesafe>mesafe2):
            print("HEDEF SEÇİMİ = AV1")
        else:
            print("HEDEF SEÇİMİ = AV2")
except KeyboardInterrupt:
    print("Takip durduruldu.")
    avci_ucak.durdur()
    av1_ucak.durdur()
    av2_ucak.durdur()
