import numpy as np
import math



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

# Test açıları (radyan cinsinden)
yaw = math.radians(45)  # 45 derece
pitch = math.radians(0)  # 0 derece
roll = math.radians(0)  # 0 derece

yon_vektor = yon_vektorunu_hesapla(yaw, pitch, roll)
print(f"Yön vektörü: {yon_vektor}")