import cv2
from decouple import config, Csv
rtsp= config('MAIN_STREAM_URL')

cap = cv2.VideoCapture(rtsp, cv2.CAP_FFMPEG)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # Small buffer for low latency

while True:
    ret, frame = cap.read()
    if not ret:
        break



    cv2.namedWindow(f"Video Stream (Source)", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(f"Video Stream (Source)", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow("RTSP Stream", frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()