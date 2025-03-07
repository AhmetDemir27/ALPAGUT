from pymavlink import mavutil
import math
import threading
import numpy as np
import time
from filterpy.kalman import KalmanFilter

class EKF_Takip_ArduPilot:
    def __init__(self, avci_conn_str, av_conn_str):
        # MAVLink bağlantılarını başlat
        self.avci_master = mavutil.mavlink_connection(avci_conn_str)  # Avcı uçağı
        self.avci_master.wait_heartbeat()
        print(f"Avcı uçağı {avci_conn_str} ile bağlandı")

        # Avcı uçağını GUIDED moduna al
        self.avci_master.mav.set_mode_send(
            self.avci_master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            self.avci_master.mode_mapping()['GUIDED']
        )

        self.av_master = mavutil.mavlink_connection(av_conn_str)  # Av uçağı
        self.av_master.wait_heartbeat()
        print(f"Av uçağı {av_conn_str} ile bağlandı")

        # EKF Tanımla (Hedefin konum ve hız kestirimi için)
        self.kf = KalmanFilter(dim_x=6, dim_z=3)  # [x, y, z, vx, vy, vz]
        self.kf.x = np.zeros(6)  # Başlangıç durum vektörü
        dt = 0.1  # Zaman adımı (10 Hz)

        # Durum geçiş matrisi (hız bileşenleri)
        self.kf.F = np.eye(6)
        for i in range(3):
            self.kf.F[i, i + 3] = dt

        # Ölçüm matrisi (x, y, z alıyoruz)
        self.kf.H = np.zeros((3, 6))
        for i in range(3):
            self.kf.H[i, i] = 1

        # Gürültü kovaryansları
        self.kf.R *= 0.5  # Ölçüm gürültüsü
        self.kf.Q *= 0.1  # Süreç gürültüsü
        self.kf.P *= 10   # Başlangıç kovaryansı

        self.running = False

    def baslat(self):
        """Takibi başlat"""
        self.running = True
        while self.running:
            hedef_konum = self.av_konum_hiz_al()
            if hedef_konum:
                self.takip_et(hedef_konum)
            time.sleep(0.1)

    def durdur(self):
        """Takibi durdur"""
        self.running = False

    def av_konum_hiz_al(self):
        """Av uçağının konum ve hız bilgisini MAVLink üzerinden al"""
        msg = self.av_master.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000
            vx = msg.vx / 100  # m/s
            vy = msg.vy / 100
            vz = msg.vz / 100

            # EKF'ye ölçüm ekle
            z = np.array([lat, lon, alt])
            self.kf.predict()
            self.kf.update(z)

            return lat, lon, alt, vx, vy, vz
        return None

    def takip_et(self, hedef_konum):
        """Avcı uçağını av uçağının gelecekteki konumuna yönlendir"""
        lat, lon, alt, vx, vy, vz = hedef_konum

        # Gelecekteki konumu tahmin etmek için zaman adımı (örneğin 5 saniye sonrası)
        tahmin_suresi = 5  # saniye

        # Gelecekteki konumu hesapla
        hedef_lat = lat + (vx * tahmin_suresi / 111320)  # 1° lat ≈ 111.32 km
        hedef_lon = lon + (vy * tahmin_suresi / (40075000 * math.cos(math.radians(lat)) / 360))
        hedef_alt = alt + vz * tahmin_suresi  # Dikey takip (isteğe bağlı)

        # Avcıyı hedef noktaya yönlendir
        self.goto_position(hedef_lat, hedef_lon, hedef_alt)

    def goto_position(self, lat, lon, alt):
        """MAVLink ile belirli bir konuma gitme komutu gönder"""
        self.avci_master.mav.mission_item_send(
            self.avci_master.target_system,
            self.avci_master.target_component,
            0,  # Sequence (0 çünkü tek bir WP ekliyoruz)
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Global Relative Altitude
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # WP komutu
            2,  # Current (0: görev, 2: doğrudan uç)
            0,  # Autocontinue
            0, 0, 0, 0,  # Parametreler (İleri hız, zaman vb.)
            float(lat),  # Latitude (float, derece)
            float(lon),  # Longitude (float, derece)
            float(alt)   # Altitude (float, relative)
        )


def haversine(lat1, lon1, lat2, lon2):
    """İki koordinat arasındaki mesafeyi hesapla (metre cinsinden)"""
    R = 6371e3  # Dünya'nın yarıçapı (metre)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # İki nokta arasındaki mesafe (metre)


def iki_vektor_arasi_aci(vektor1, vektor2):
    """İki yön vektörü arasındaki açıyı hesaplar (derece cinsinden)."""
    v1 = np.array(vektor1)
    v2 = np.array(vektor2)
    dot_product = np.dot(v1, v2)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
    return math.degrees(math.acos(cos_theta))


def yon_vektorunu_hesapla(yaw, pitch, roll):
    """Yaw, pitch ve roll açılarından yön vektörünü hesapla."""
    Ry = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    Rp = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    Rr = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    R = np.dot(Rr, np.dot(Rp, Ry))
    return np.dot(R, np.array([1, 0, 0]))


def hedef_konum_acisi_hesapla(avci_konum, avci_yon_vektor, hedef_konum):
    """Avcı uçağın yön vektörü ile hedef uçağın konumu arasındaki açıyı hesaplar."""
    hedef_vektor = np.array([hedef_konum['lat'] - avci_konum['lat'],
                             hedef_konum['lon'] - avci_konum['lon'],
                             0])  # Z eksenini ihmal ediyoruz
    return iki_vektor_arasi_aci(avci_yon_vektor, hedef_vektor)


def calculate_score(mesafe, hiz, vektor, hedef_acisi, w1=0.4, w2=0.2, w3=0.2, w4=0.2):
    """Hedef seçimi için puan hesapla."""
    mesafe_puan = 100 - ((100 * mesafe) / 500)
    hiz_puan = 100 - ((100 * hiz) / 35)
    vektor_puan = 100 - (vektor / 180 * 100)
    konum_puan = 100 if hedef_acisi < 45 else 50 if hedef_acisi < 90 else 0
    return w1 * mesafe_puan + w2 * hiz_puan + w3 * vektor_puan + w4 * konum_puan, mesafe_puan, hiz_puan, vektor_puan, konum_puan


def choose_target(targets):
    """En uygun hedefi seç."""
    scores = [calculate_score(distance, speed, vektor, hedef_acisi) for distance, speed, vektor, hedef_acisi in targets]
    max_puan = max(scores, key=lambda x: x[0])
    return scores.index(max_puan), max_puan


class Ucak:
    def __init__(self, connection_str):
        self.connection_str = connection_str
        self.master = mavutil.mavlink_connection(connection_str)
        self.master.wait_heartbeat()
        print(f"{connection_str} ile bağlandı")
        self.running = False
        self.position = {'lat': None, 'lon': None, 'alt': None, 'speed': None, 'yon_vektor': None}
        self.konum_thread = None

    def konum_hiz_aci_bilgisi_al(self):
        """Uçağın konum, hız ve yön bilgilerini al."""
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

            msg1 = self.master.recv_match(type='ATTITUDE', blocking=True)
            if msg1:
                yaw = msg1.yaw
                pitch = msg1.pitch
                roll = msg1.roll
                self.position['yon_vektor'] = yon_vektorunu_hesapla(yaw, pitch, roll)

    def start_konum_takip(self):
        """Konum takip thread'ini başlat."""
        if not self.running:
            self.running = True
            self.konum_thread = threading.Thread(target=self.konum_hiz_aci_bilgisi_al, daemon=True)
            self.konum_thread.start()

    def durdur(self):
        """Konum takip thread'ini durdur."""
        self.running = False
        if self.konum_thread:
            self.konum_thread.join()


# Uçakları başlat
avci_ucak = Ucak('127.0.0.1:14550')
av1_ucak = Ucak('127.0.0.1:14560')
av2_ucak = Ucak('127.0.0.1:14570')
av3_ucak = Ucak('127.0.0.1:14580')
av4_ucak = Ucak('127.0.0.1:14590')

ucak_listesi = [av1_ucak, av2_ucak, av3_ucak, av4_ucak]

# Konum bilgisi takip başlat
avci_ucak.start_konum_takip()
for ucak in ucak_listesi:
    ucak.start_konum_takip()

# Anlık mesafe ölçümü ve hedef seçimi
try:
    while True:
        targets = []
        for i, ucak in enumerate(ucak_listesi):
            if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], ucak.position['lat'], ucak.position['lon']) and avci_ucak.position['yon_vektor'] is not None and ucak.position['yon_vektor'] is not None:
                av_speed = ucak.position["speed"]
                av_mesafe = haversine(avci_ucak.position['lat'], avci_ucak.position['lon'], ucak.position['lat'], ucak.position['lon'])
                av_vektor_aci = iki_vektor_arasi_aci(avci_ucak.position['yon_vektor'], ucak.position['yon_vektor'])
                hedef_acisi = hedef_konum_acisi_hesapla(avci_ucak.position, avci_ucak.position['yon_vektor'], ucak.position)
                targets.append([av_mesafe, av_speed, av_vektor_aci, hedef_acisi])

        # Hedef seçimi
        if targets:
            best_target_index, best_target_score = choose_target(targets)
            best_target_connection_str = ucak_listesi[best_target_index].connection_str
            avci_connection_str = avci_ucak.connection_str
            print(f"En uygun hedef: {best_target_index + 1}  Puanı: {best_target_score[0]:.2f}")
            print(f"En uygun hedefin bağlantı string'i: {best_target_connection_str}")
            print(f"Avcı uçağın bağlantı string'i: {avci_connection_str}\n")

            if best_target_score[0] > 70:
                print(f"En uygun hedef: {best_target_index + 1}  Puanı: {best_target_score[0]:.2f}")
                print(f"En uygun hedefin bağlantı string'i: {best_target_connection_str}")
                print(f"Avcı uçağın bağlantı string'i: {avci_connection_str}\n")
                break
        time.sleep(0.2)

except KeyboardInterrupt:
    print("Takip durduruldu.")
    avci_ucak.durdur()
    for ucak in ucak_listesi:
        ucak.durdur()


# Takip başlat
takipci = EKF_Takip_ArduPilot(avci_conn_str="udp:" + avci_connection_str, av_conn_str="udp:" + best_target_connection_str)
try:
    takipci.baslat()
except KeyboardInterrupt:
    takipci.durdur()
