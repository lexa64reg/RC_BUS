import asyncio
import cv2
import numpy as np
import pyaudio
import ffmpeg
import threading
from queue import Queue
import time

# Настройки
MAIN_STREAM_URL = "rtsp://admin:GLCCRT@192.168.99.41:554/h264"  # Основной RTSP-поток
PIP_STREAM_URL = "rtsp://lexa64reg:Sar270286@192.168.99.50:554/channel=2_stream=1"   # PIP RTSP-поток
MAIN_WINDOW_NAME = "Main Stream"
PIP_SCALE = 0.3  # Масштаб PIP-потока (30%)
PIP_POSITION = (1, 1)  # Позиция PIP (x, y)
FPS = 30  # Частота кадров
AUDIO_RATE = 44100  # Частота дискретизации аудио

# Глобальные переменные для управления потоками
main_audio_queue = Queue()
pip_audio_queue = Queue()
stop_audio_threads = threading.Event()

def setup_video(main_url, pip_url):
    # Инициализация видеопотоков
    main_stream = cv2.VideoCapture(main_url)
    pip_stream = cv2.VideoCapture(pip_url) if pip_url else None
    
    if not main_stream.isOpened():
        print("Ошибка: не удалось открыть основной видеопоток")
        return None, None
    if pip_stream and not pip_stream.isOpened():
        print("Ошибка: не удалось открыть PIP-видеопоток")
        pip_stream = None
    
    # Установить полноэкранный режим
    cv2.namedWindow(MAIN_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(MAIN_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    
    return main_stream, pip_stream

def setup_audio():
    # Инициализация PyAudio
    p = pyaudio.PyAudio()
    main_stream = p.open(format=pyaudio.paInt16, channels=2, rate=AUDIO_RATE, output=True)
    pip_stream = p.open(format=pyaudio.paInt16, channels=2, rate=AUDIO_RATE, output=True)
    
    return p, main_stream, pip_stream

def stream_audio(url, queue, stream_name):
    try:
        process = (
            ffmpeg
            .input(url, rtsp_transport="tcp")
            .output("pipe:", format="s16le", acodec="pcm_s16le", ar=AUDIO_RATE, ac=2)
            .run_async(pipe_stdout=True)
        )
        while not stop_audio_threads.is_set():
            audio_data = process.stdout.read(4096)
            if not audio_data:
                break
            queue.put(audio_data)
    except Exception as e:
        print(f"Ошибка в потоке аудио {stream_name}: {e}")
    finally:
        process.stdout.close()

def play_audio(audio_stream, audio_queue):
    while not stop_audio_threads.is_set():
        if not audio_queue.empty():
            audio_data = audio_queue.get()
            audio_stream.write(audio_data)
        time.sleep(0.01)

def update_loop(main_stream, pip_stream):
    ret_main, frame_main = main_stream.read()
    ret_pip, frame_pip = pip_stream.read() if pip_stream else (False, None)
    
    if not ret_main:
        print("Ошибка: не удалось прочитать кадр из основного потока")
        return False, None
    
    # Получить размеры основного кадра
    h_main, w_main = frame_main.shape[:2]
    
    if ret_pip and frame_pip is not None:
        # Изменить размер PIP-кадра
        h_pip, w_pip = frame_pip.shape[:2]
        new_h_pip = int(h_pip * PIP_SCALE)
        new_w_pip = int(w_pip * PIP_SCALE)
        frame_pip = cv2.resize(frame_pip, (new_w_pip, new_h_pip))
        
        # Наложить PIP-кадр
        x, y = PIP_POSITION
        if x + new_w_pip <= w_main and y + new_h_pip <= h_main:
            frame_main[y:y+new_h_pip, x:x+new_w_pip] = frame_pip
    
    # Показать кадр
    cv2.imshow(MAIN_WINDOW_NAME, frame_main)
    key = cv2.waitKey(1) & 0xFF
    return True, key

async def main():
    # Начальные URL-адреса
    current_main_url = MAIN_STREAM_URL
    current_pip_url = PIP_STREAM_URL
    
    # Настройка видео и аудио
    main_stream, pip_stream = setup_video(current_main_url, current_pip_url)
    if main_stream is None:
        return
    
    p, main_audio_stream, pip_audio_stream = setup_audio()
    
    # Запуск аудиопотоков
    main_audio_thread = threading.Thread(target=stream_audio, args=(current_main_url, main_audio_queue, "main"), daemon=True)
    pip_audio_thread = threading.Thread(target=stream_audio, args=(current_pip_url, pip_audio_queue, "pip"), daemon=True) if pip_stream else None
    main_audio_play_thread = threading.Thread(target=play_audio, args=(main_audio_stream, main_audio_queue), daemon=True)
    pip_audio_play_thread = threading.Thread(target=play_audio, args=(pip_audio_stream, pip_audio_queue), daemon=True) if pip_stream else None
    
    main_audio_thread.start()
    if pip_audio_thread:
        pip_audio_thread.start()
    main_audio_play_thread.start()
    if pip_audio_play_thread:
        pip_audio_play_thread.start()
    
    try:
        while True:
            success, key = update_loop(main_stream, pip_stream)
            if not success:
                break
                
            # Обработка клавиш
            if key == ord('q'):
                break
            elif key == ord('v'):
                # Смена потоков
                stop_audio_threads.set()  # Остановить текущие аудиопотоки
                main_stream.release()
                if pip_stream:
                    pip_stream.release()
                main_audio_stream.stop_stream()
                if pip_stream:
                    pip_audio_stream.stop_stream()
                
                # Поменять местами URL
                current_main_url, current_pip_url = current_pip_url, current_main_url
                
                # Перезапустить видео и аудио
                main_stream, pip_stream = setup_video(current_main_url, current_pip_url)
                if main_stream is None:
                    break
                
                # Очистить очереди и перезапустить аудиопотоки
                main_audio_queue.queue.clear()
                pip_audio_queue.queue.clear()
                stop_audio_threads.clear()
                
                main_audio_thread = threading.Thread(target=stream_audio, args=(current_main_url, main_audio_queue, "main"), daemon=True)
                pip_audio_thread = threading.Thread(target=stream_audio, args=(current_pip_url, pip_audio_queue, "pip"), daemon=True) if pip_stream else None
                main_audio_thread.start()
                if pip_audio_thread:
                    pip_audio_thread.start()
            
            await asyncio.sleep(1.0 / FPS)
    finally:
        # Очистка
        stop_audio_threads.set()
        main_stream.release()
        if pip_stream:
            pip_stream.release()
        main_audio_stream.stop_stream()
        main_audio_stream.close()
        if pip_stream:
            pip_audio_stream.stop_stream()
            pip_audio_stream.close()
        p.terminate()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    asyncio.run(main())