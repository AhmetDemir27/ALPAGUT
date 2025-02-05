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

def hedef_konum_acisi_hesapla(avci_konum, avci_yon_vektor, hedef_konum):
    """
    Avcı uçağın yön vektörü ile hedef uçağın konumu arasındaki açıyı hesaplar.
    """
    # Avcı uçağın konumu ve hedef uçağın konumu arasındaki vektör
    hedef_vektor = np.array([hedef_konum['lat'] - avci_konum['lat'],
                             hedef_konum['lon'] - avci_konum['lon'],
                             0])  # Z eksenini ihmal ediyoruz
    
    # Yön vektörü ile hedef vektörü arasındaki açıyı hesapla
    acı = iki_vektor_arasi_aci(avci_yon_vektor, hedef_vektor)
    
    return acı

def calculate_score(mesafe, hiz, vektor, hedef_acisi, w1=0.5, w2=0.2, w3=0.2, w4=0.1):
    mesafe_puan = 100 - ((100 * mesafe) / 500)
    hiz_puan = 100 - ((100 * hiz) / 35)
    vektor_puan = 100 - (vektor / 180 * 100)
    
    # Hedefin önümüzde mi arkada mı olduğunu belirleme
    if hedef_acisi < 45:
        konum_puan = 100  # Önümüzde
    elif hedef_acisi<90:
        konum_puan= 50
    else:
        konum_puan = 0  # Arkada
    
    score = w1 * mesafe_puan + w2 * hiz_puan + w3 * vektor_puan + w4 * konum_puan
    return score, mesafe_puan, hiz_puan, vektor_puan, konum_puan

def choose_target(targets):
    scores = [calculate_score(distance, speed, vektor, hedef_acisi) for distance, speed, vektor, hedef_acisi in targets]
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

ucak_listesi = [av1_ucak, av2_ucak, av3_ucak, av4_ucak]

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

        for i in range(len(ucak_listesi)):
            if None not in (avci_ucak.position['lat'], avci_ucak.position['lon'], ucak_listesi[i].position['lat'], ucak_listesi[i].position['lon']) and avci_ucak.position['yon_vektor'] is not None and ucak_listesi[i].position['yon_vektor'] is not None:
                
                # Dinamik değişken atamaları
                av_speed = ucak_listesi[i].position["speed"]
                av_mesafe = haversine(avci_ucak.position['lat'], avci_ucak.position['lon'], ucak_listesi[i].position['lat'], ucak_listesi[i].position['lon'])
                av_vektor_aci = iki_vektor_arasi_aci(avci_ucak.position['yon_vektor'], ucak_listesi[i].position['yon_vektor'])

                # Hedefin önümüzde mi arkada mı olduğunu belirleme
                hedef_acisi = hedef_konum_acisi_hesapla(avci_ucak.position, avci_ucak.position['yon_vektor'], ucak_listesi[i].position)
                
                # Hız, mesafe ve vektör açılarını yazdırma
                print(f"Av{i+1} Hızı: {av_speed:.2f}")
                print(f"Av{i+1} ile arasındaki mesafe: {av_mesafe:.2f} metre")
                print(f"Av{i+1} ile arasındaki vektör açısı: {av_vektor_aci:.2f} derece")
                print(f"Av{i+1} önümüzde mi: {'Evet' if hedef_acisi < 90 else 'Hayır'}\n")
                
                # targets listesine ekleme
                targets.append([av_mesafe, av_speed, av_vektor_aci, hedef_acisi])

        # Hedef seçimi
        if targets:
            best_target = choose_target(targets)
            print(f"En uygun hedef: Av {best_target[0] + 1}  Puanı: {best_target[1][0]:.2f}\n")
        time.sleep(0.2)
except KeyboardInterrupt:
    print("Takip durduruldu.")
    avci_ucak.durdur()
    av1_ucak.durdur()
    av2_ucak.durdur()
    av3_ucak.durdur()
    av4_ucak.durdur()
