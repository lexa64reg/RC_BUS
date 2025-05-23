import cv2
import os
import logging
from decouple import config, Csv
rtsp= config('MAIN_STREAM_URL')

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# RTSP configuration (use environment variables for security)
RTSP_URL = os.getenv("RTSP_URL", rtsp)

try:
    # Initialize video capture with FFMPEG backend
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    
    # Check if the stream opened successfully
    if not cap.isOpened():
        logger.error("Failed to open RTSP stream. Check URL, credentials, or FFMPEG support.")
        exit(1)
    
    # Attempt to set buffer size (may not work with all backends)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Small buffer for low latency
    cap.set(cv2.CAP_PROP_FPS, 5)
    logger.info("RTSP stream opened successfully.")

    while True:
        ret, frame = cap.read()
        if not ret:
            logger.warning("Failed to read frame. Attempting to reconnect...")
            cap.release()
            cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                logger.error("Reconnection failed. Exiting.")
                break
            continue
        
        # Optional: Resize frame for better performance (e.g., 50% of original size)
        # frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        
        cv2.imshow("RTSP Stream", frame)
        
        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord("q"):
            logger.info("Stream terminated by user.")
            break

except Exception as e:
    logger.error(f"An error occurred: {str(e)}")

finally:
    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    logger.info("Resources released and windows closed.")