from pymavlink import mavutil
import math
import threading
import numpy as np
import time

def haversine(lat1, lon1, lat2, lon2):
    R = 6371e3  # Dünya'nın yarıçapı (metre)
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c  # İki nokta arasındaki mesafe (metre)

def iki_vektor_arasi_aci(vektor1, vektor2):
    """
    İki yön vektörü arasındaki açıyı hesaplar (derece cinsinden).
    vektor1 ve vektor2 formatı: (x, y, z)
    """
    # NumPy array'e çevir
    v1 = np.array(vektor1)
    v2 = np.array(vektor2)
    
    # İç çarpım (dot product)
    dot_product = np.dot(v1, v2)
    
    # Vektörlerin büyüklükleri (norm)
    norm_v1 = np.linalg.norm(v1)
    norm_v2 = np.linalg.norm(v2)
    
    # Kosinüs teoremi ile açı hesaplama
    cos_theta = dot_product / (norm_v1 * norm_v2)
    
    # Değer aralığını kontrol et (hataya karşı koruma)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    
    # Açıyı radyandan dereceye çevir
    angle = math.degrees(math.acos(cos_theta))
    
    return angle

def yon_vektorunu_hesapla(yaw, pitch, roll):
    # Yaw (sapma) dönüş matrisi
    Ry = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    
    # Pitch (yunuslama) dönüş matrisi
    Rp = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0,            1,            0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    # Roll (yuvarlanma) dönüş matrisi
    Rr = np.array([
        [1,            0,             0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])
    
    # Toplam dönüş matrisi
    R = np.dot(Rr, np.dot(Rp, Ry))
    
    # Başlangıç yön vektörü (ileri yön)
    v0 = np.array([1, 0, 0])
    
    # Yön vektörünü hesapla
    yon_vektor = np.dot(R, v0)
    
    return yon_vektor

def calculate_score(mesafe, hiz, vektor, w1=0.6, w2=0.2, w3=0.2):
    mesafe_puan = 100 - ((100 * mesafe) / 500)
    hiz_puan = 100 - ((100 * hiz) / 35)
    vektor_puan = 100 - (vektor / 180 * 100)
    score = w1 * mesafe_puan + w2 * hiz_puan + w3 * vektor_puan
    return score, mesafe_puan, hiz_puan, vektor_puan

def choose_target(targets):
    scores = [calculate_score(distance, speed, vektor) for distance, speed, vektor in targets]
    max_puan = max(scores, key=lambda x: x[0])  # En yüksek puanı alan hedef
    best_target_index = scores.index(max_puan)
    return best_target_index, max_puan

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
                yaw = msg1.yaw  # Yaw açısı (radyan)
                pitch = msg1.pitch  # Pitch açısı (radyan)
                roll = msg1.roll  # Roll açısı (radyan)
                self.position['yon_vektor'] = yon_vektorunu_hesapla(yaw, pitch, roll)

    def start_konum_takip(self):
        if not self.running:
            self.running = True
            self.konum_thread = threading.Thread(target=self.konum_hiz_aci_bilgisi_al, daemon=True)
            self.konum_thread.start()
    
    def durdur(self):
        self.running = False
        if self.konum_thread:
            self.konum_thread.join()

# Uçakları başlat
avci_ucak = Ucak('127.0.0.1:14550')
av1_ucak = Ucak('127.0.0.1:14560')
av2_ucak = Ucak('127.0.0.1:14570')
av3_ucak = Ucak('127.0.0.1:14580')
av4_ucak = Ucak('127.0.0.1:14590')

# Konum bilgisi takip başlat
avci_ucak.start_konum_takip()
av1_ucak.start_konum_takip()
av2_ucak.start_konum_takip()
av3_ucak.start_konum_takip()
av4_ucak.start_konum_takip()

# Anlık mesafe ölçümü
try:
    while True:
        targets = []

        # Av1 hedefi
        if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], av1_ucak.position['lat'], av1_ucak.position['lon']) and avci_ucak.position['yon_vektor'] is not None and av1_ucak.position['yon_vektor'] is not None:
            mesafe = haversine(avci_ucak.position['lat'], avci_ucak.position['lon'], av1_ucak.position['lat'], av1_ucak.position['lon'])
            vektor_aci = iki_vektor_arasi_aci(avci_ucak.position['yon_vektor'], av1_ucak.position['yon_vektor'])
            av1_speed = av1_ucak.position['speed']
            print(f"Av1 in Hızı {av1_speed:.2f}")
            print(f"Av1_ucak ile arasındaki mesafe: {mesafe:.2f} metre")
            print(f"Av1_ucak ile arasındaki vektor acısı: {vektor_aci:.2f} derece\n")
            targets.append([mesafe, av1_speed, vektor_aci])

        # Av2 hedefi
        if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], av2_ucak.position['lat'], av2_ucak.position['lon']) and avci_ucak.position['yon_vektor'] is not None and av2_ucak.position['yon_vektor'] is not None:
            mesafe2 = haversine(avci_ucak.position['lat'], avci_ucak.position['lon'], av2_ucak.position['lat'], av2_ucak.position['lon'])
            vektor_aci2 = iki_vektor_arasi_aci(avci_ucak.position['yon_vektor'], av2_ucak.position['yon_vektor'])
            av2_speed = av2_ucak.position['speed']
            print(f"Av2 in Hızı: {av2_speed:.2f}")
            print(f"Av2_ucak ile arasındaki mesafe: {mesafe2:.2f} metre")
            print(f"Av2_ucak ile arasındaki vektor acısı: {vektor_aci2:.2f} derece\n")
            targets.append([mesafe2, av2_speed, vektor_aci2])

        # Av3 hedefi
        if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], av3_ucak.position['lat'], av3_ucak.position['lon']) and avci_ucak.position['yon_vektor'] is not None and av3_ucak.position['yon_vektor'] is not None:
            mesafe3 = haversine(avci_ucak.position['lat'], avci_ucak.position['lon'], av3_ucak.position['lat'], av3_ucak.position['lon'])
            vektor_aci3 = iki_vektor_arasi_aci(avci_ucak.position['yon_vektor'], av3_ucak.position['yon_vektor'])
            av3_speed = av3_ucak.position['speed']
            print(f"Av3 in Hızı {av3_speed:.2f}")
            print(f"Av3_ucak ile arasındaki mesafe: {mesafe3:.2f} metre")
            print(f"Av3_ucak ile arasındaki vektor acısı: {vektor_aci3:.2f} derece\n")
            targets.append([mesafe3, av3_speed, vektor_aci3])

        # Av4 hedefi
        if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], av4_ucak.position['lat'], av4_ucak.position['lon']) and avci_ucak.position['yon_vektor'] is not None and av4_ucak.position['yon_vektor'] is not None:
            mesafe4 = haversine(avci_ucak.position['lat'], avci_ucak.position['lon'], av4_ucak.position['lat'], av4_ucak.position['lon'])
            vektor_aci4 = iki_vektor_arasi_aci(avci_ucak.position['yon_vektor'], av4_ucak.position['yon_vektor'])
            av4_speed = av4_ucak.position['speed']
            print(f"Av4 in Hızı: {av4_speed:.2f}")
            print(f"Av4_ucak ile arasındaki mesafe: {mesafe4:.2f} metre")
            print(f"Av4_ucak ile arasındaki vektor acısı: {vektor_aci4:.2f} derece\n")
            targets.append([mesafe4, av4_speed, vektor_aci4])

        # Hedef seçimi
        if targets:
            best_target = choose_target(targets)
            print(f"En uygun hedef: Av {best_target[0] + 1}  Puanı: {best_target[1][0]:.2f}\n")
        time.sleep(1)

except KeyboardInterrupt:
    print("Takip durduruldu.")
    avci_ucak.durdur()
    av1_ucak.durdur()
    av2_ucak.durdur()
    av3_ucak.durdur()
    av4_ucak.durdur()