from pymavlink import mavutil
import sys
import threading
import math
import time


class Ucak:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print(f"{connection_str} ile bağlandı")
        self.running = True

    def mod_degistir(self, mode):
        if mode not in self.master.mode_mapping():
            print('Bilinmeyen mod:', mode)
            print('Mevcut modlar:', list(self.master.mode_mapping().keys()))
            return

        mode_id = self.master.mode_mapping()[mode]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"Mod {mode} olarak değiştirildi")

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
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
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

    def konum_bilgisi_al_surekli(self):
        print("Sürekli konum bilgisi alınıyor...")

        def konum_dongusu():
            while self.running:
                msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
                if msg:
                    altitude = msg.relative_alt / 1000  # Yükseklik (metre)
                    x_position = msg.lat / 1e7          # Enlem (derece)
                    y_position = msg.lon / 1e7          # Boylam (derece)
                    x_speed = msg.vx
                    y_speed = msg.vy
                    z_speed = msg.vz
                    speed = math.sqrt((x_speed / 100) ** 2 + (y_speed / 100) ** 2 + (z_speed / 100) ** 2)

                    print(f"Yükseklik: {altitude} m, Enlem: {x_position}°, Boylam: {y_position}°, Hız: {speed:.2f} m/s")

        self.running = True
        self.konum_thread = threading.Thread(target=konum_dongusu)
        self.konum_thread.start()

    def durdur(self):
        self.running = False
        if hasattr(self, 'konum_thread'):
            self.konum_thread.join()
        print("Sürekli konum bilgisi alma işlemi durduruldu.")


# Uçakları başlat
avci_ucak = Ucak('127.0.0.1:14550')
av1_ucak = Ucak('127.0.0.1:14560')
av2_ucak = Ucak('127.0.0.1:14570')
#av3_ucak = Ucak('127.0.0.1:14580')
#av4_ucak = Ucak('127.0.0.1:14590')

# Uçak seçme fonksiyonu
def ucak_secim_fonksiyonu():
    while True:
        print("\nUCAKLAR:")
        print("0 - Avcı Uçak")
        print("1 - Av Uçak 1")
        print("2 - Av Uçak 2")
        print("3 - Av Uçak 3")
        print("4 - Av Uçak 4")
        print("q - Çıkış")

        ucak_secim = input("Uçağınızı seçiniz: ")
        if ucak_secim == "0":
            return avci_ucak
        elif ucak_secim == "1":
            return av1_ucak
        elif ucak_secim == "2":
            return av2_ucak
#        elif ucak_secim == "3":
#            return av3_ucak
#        elif ucak_secim == "4":
#            return av4_ucak
        elif ucak_secim == 'q':
            print("Sistem Kapatılıyor")
            sys.exit()
        else:
            print("Yanlış seçim yaptınız, lütfen tekrar deneyiniz")


# Ana kontrol döngüsü
def komut_kontrol(kullanici_ucak):
    try:
        while True:
            print("\nKomutlar:")
            print("1 - Mod Değiştir")
            print("2 - Motor Arm")
            print("3 - Motor Disarm")
            print("4 - İniş Yap")
            print("5 - Sürekli Konum ve Hız Bilgisi Al")
            print("6 - Uçak seçimi yap")
            print("0 - Çıkış")

            secim = input("Seçiminizi yapın: ")

            if secim == "1":
                yeni_mod = input("Yeni mod adı: ")
                kullanici_ucak.mod_degistir(yeni_mod)
            elif secim == "2":
                kullanici_ucak.motor_arm()
            elif secim == "3":
                kullanici_ucak.motor_disarm()
            elif secim == "4":
                kullanici_ucak.inis_yap()
            elif secim == "5":
                kullanici_ucak.konum_bilgisi_al_surekli()
                input("Durdurmak için Enter'a basın...")
                kullanici_ucak.durdur()
            elif secim == "6":
                print("Yeni uçak seçiliyor...")
                kullanici_ucak = ucak_secim_fonksiyonu()
            elif secim == "0":
                print("Sistem Kapatılıyor...")
                sys.exit()
            else:
                print("Geçersiz seçim. Tekrar deneyin.")
    except KeyboardInterrupt:
        print("\nProgram durduruldu.")
        kullanici_ucak.durdur()

if __name__ == "__main__":
    while True:
        secilen_ucak = ucak_secim_fonksiyonu()
        while True:
            komut_kontrol(secilen_ucak)