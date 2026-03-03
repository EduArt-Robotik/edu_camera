import cv2
import time
import os

# Kamera-Index (0 = Standardkamera)
camera_index = 0
output_dir = "snapshots"
os.makedirs(output_dir, exist_ok=True)

cap = cv2.VideoCapture(camera_index)
if not cap.isOpened():
    print("Kamera konnte nicht geöffnet werden.")
    exit(1)

try:
    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Fehler beim Lesen des Kamerabildes.")
            break
        filename = os.path.join(output_dir, f"snapshot_{count:04d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"Bild gespeichert: {filename}")
        count += 1
        time.sleep(1)
except KeyboardInterrupt:
    print("Aufnahme beendet.")
finally:
    cap.release()
