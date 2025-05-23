import cv2
import numpy as np
import time
from decouple import config, Csv

#Список URL-адресов видеопотоков
video_sources = config('video_sources', cast=Csv()) 

# Текущий индекс потока 0
current_source = 0

def main():
    global current_source
    
    # Функция для корректного открытия потока
    def open_stream(source_index):
        cap = cv2.VideoCapture(video_sources[source_index])
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
        time.sleep(1)  # Ждем инициализации
        
        # Проверяем успешность открытия
        if not cap.isOpened():
            print(f"Ошибка при открытии потока {source_index + 1}")
            return None
        
        return cap

    cap = open_stream(current_source)
    if cap is None:
        print("Не удалось открыть начальный поток")
        return

    try:
        while True:
            # Чтение кадра
            if cap is not None:
                ret, frame = cap.read()
                
                if not ret:
                    print(f"Ошибка чтения кадра или поток {current_source + 1} закончился")
                    # Пытаемся переподключиться
                    cap.release()
                    cap = open_stream(current_source)
                    if cap is None:
                        print("Не удалось переподключиться")
                        break
                    continue
                
                # Отображение кадра
                cv2.namedWindow(f"Video Stream (Source {current_source + 1})", cv2.WND_PROP_FULLSCREEN)
                cv2.setWindowProperty(f"Video Stream (Source {current_source + 1})", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                cv2.imshow(f"Video Stream (Source {current_source + 1})", frame)

                
                # Обработка нажатий клавиш
                key = cv2.waitKey(1)
                if key == ord('v'):  # Переключение потока
                    current_source = (current_source + 1) % len(video_sources)
                    
                    # Закрываем текущий поток, если он есть
                    if cap is not None:
                        cap.release()
                        cap = None
                        cv2.destroyAllWindows()
                    
                    # Открываем новый
                    cap = open_stream(current_source)
                    if cap is None:
                        print("Не удалось открыть новый поток")
                        break
                    continue
                    
                    print(f"Переключено на источник: {current_source + 1}")
                elif key == 27:  # Esc - выход
                    break
                    
    except Exception as e:
        print(f"Произошла ошибка: {e}")
        
    finally:
        # Освобождение ресурсов
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
