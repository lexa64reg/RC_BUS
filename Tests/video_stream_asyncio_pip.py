import asyncio
import cv2
import numpy as np
import platform

# Настройки
MAIN_WINDOW_NAME = "Main Stream"
PIP_SCALE = 0.3  # Масштаб второго потока (30% от оригинального размера)
PIP_POSITION = (10, 10)  # Позиция второго потока (x, y) в углу
FPS = 30  # Частота кадров

def setup():
    # Для RTSP (локальное окружение):
    # main_stream = cv2.VideoCapture("rtsp://your_rtsp_url1")
    # pip_stream = cv2.VideoCapture("rtsp://your_rtsp_url2")
    
    # Для Pyodide или тестирования используем веб-камеру или видеофайлы
    main_stream = cv2.VideoCapture("rtsp://admin:Sar270286@192.168.88.169:554/profile3")  # Основной поток (веб-камера)
    pip_stream = cv2.VideoCapture("rtsp://lexa64reg:Sar270286@192.168.99.50:554/channel=2_stream=1")   # Второй поток (вторая веб-камера, если есть)
    
    # Проверка, что потоки открыты
    if not main_stream.isOpened():
        print("Ошибка: не удалось открыть основной поток")
        return None, None
    if not pip_stream.isOpened():
        print("Ошибка: не удалось открыть PIP-поток")
        return main_stream, None
    
    # Установить режим полного экрана для основного окна
    cv2.namedWindow(MAIN_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty(MAIN_WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    
    return main_stream, pip_stream

def update_loop(main_stream, pip_stream):
    # Чтение кадров
    ret_main, frame_main = main_stream.read()
    ret_pip, frame_pip = pip_stream.read() if pip_stream else (False, None)
    
    if not ret_main:
        print("Ошибка: не удалось прочитать кадр из основного потока")
        return
    
    # Получить размеры основного кадра
    h_main, w_main = frame_main.shape[:2]
    
    if ret_pip and frame_pip is not None:
        # Изменить размер PIP-кадра
        h_pip, w_pip = frame_pip.shape[:2]
        new_h_pip = int(h_pip * PIP_SCALE)
        new_w_pip = int(w_pip * PIP_SCALE)
        frame_pip = cv2.resize(frame_pip, (new_w_pip, new_h_pip))
        
        # Наложить PIP-кадр на основной
        x, y = PIP_POSITION
        if x + new_w_pip <= w_main and y + new_h_pip <= h_main:
            frame_main[y:y+new_h_pip, x:x+new_w_pip] = frame_pip
    
    # Показать кадр
    cv2.imshow(MAIN_WINDOW_NAME, frame_main)
    cv2.waitKey(1)  # Необходимо для обновления окна

async def main():
    main_stream, pip_stream = setup()
    if main_stream is None:
        return
    
    try:
        while True:
            update_loop(main_stream, pip_stream)
            await asyncio.sleep(1.0 / FPS)
    finally:
        # Освободить ресурсы
        main_stream.release()
        if pip_stream:
            pip_stream.release()
        cv2.destroyAllWindows()

if platform.system() == "Emscripten":
    asyncio.ensure_future(main())
else:
    if __name__ == "__main__":
        asyncio.run(main())