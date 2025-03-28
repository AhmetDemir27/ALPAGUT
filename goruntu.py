import cv2
from ultralytics import YOLO

# Modeli yükle ve GPU’ya taşı
model = YOLO("/home/ahmet/ALPAGUT/alpagut_code/goruntu_isleme/best.pt")
model.to("cuda")

# Kamera ayarları: V4L2 backend kullanımı
cv2.namedWindow("preview")
vc = cv2.VideoCapture(0, cv2.CAP_V4L2)

if not vc.isOpened():
    print("Kamera açılamadı!")
    exit()

while True:
    ret, frame = vc.read()
    if not ret:
        break
    
    # Eğer gerekliyse frame boyutunu düşürebilirsin
    frame = cv2.resize(frame, (640, 480))
    
    # YOLO ile tespit yap (GPU kullanarak)
    results = model.predict(frame, conf=0.5, verbose=False, device="cuda")
    
    # Tespit sonuçlarını işleyerek görselleştir
    annotated_frame = results[0].plot()
    
    # Görüntüyü göster
    cv2.imshow("preview", annotated_frame)
    
    # ESC tuşu ile çıkış (bekleme süresi 1ms)
    if cv2.waitKey(1) == 27:
        break

vc.release()
cv2.destroyAllWindows()
