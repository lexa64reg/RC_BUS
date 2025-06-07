import cv2
import time
from decouple import config, Csv

# Загрузка источников видео из конфигурации
try:
    video_sources = config('video_sources', cast=Csv())
    if not video_sources:
        raise ValueError("Список источников видео пуст")
except Exception as e:
    print(f"Ошибка загрузки источников видео: {e}")
    exit(1)

# Текущий индекс потока
current_source = 0
# Максимальное количество попыток подключения к одному источнику
MAX_RETRIES = 1

def open_stream(source_index):
    """Открывает видеопоток по указанному индексу."""
    try:
        cap = cv2.VideoCapture(video_sources[source_index])
        if not cap.isOpened():
            print(f"Ошибка открытия потока {source_index + 1}: {video_sources[source_index]}")
            return None
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)  # Установка минимального буфера (зависит от бэкенда)
        time.sleep(1)  # Ожидание инициализации потока
        return cap
    except Exception as e:
        print(f"Ошибка инициализации потока {source_index + 1}: {e}")
        return None

def main():
    """Основная функция для отображения и переключения видеопотоков."""
    global current_source
    
    # Попытка открыть начальный поток
    cap = open_stream(current_source)
    window_name = None
    
    # Создание окна, если поток открыт
    if cap is not None:
        window_name = f"Video Stream (Source {current_source + 1})"
        cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:
            # Проверяем, есть ли активный поток
            if cap is None:
                print(f"Поток {current_source + 1} недоступен, пробуем следующий...")
                retry_count = 0
                original_source = current_source
                
                # Пытаемся найти доступный источник
                while retry_count < len(video_sources):
                    current_source = (current_source + 1) % len(video_sources)
                    print(f"Попытка подключения к источнику {current_source + 1}")
                    cap = open_stream(current_source)
                    if cap is not None:
                        # Создаем новое окно только если поток открыт
                        if window_name is not None:
                            try:
                                cv2.destroyWindow(window_name)
                            except cv2.error:
                                pass  # Игнорируем ошибку, если окно не существует
                        window_name = f"Video Stream (Source {current_source + 1})"
                        cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
                        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                        break
                    retry_count += 1
                    if retry_count == len(video_sources):
                        print("Все источники недоступны, начинаем с первого...")
                        current_source = 0
                        retry_count = 0
                        time.sleep(2)  # Пауза перед повторной попыткой
                    continue
                
                if cap is None:
                    continue  # Продолжаем цикл, если не удалось подключиться

            # Чтение кадра
            ret, frame = cap.read()
            if not ret:
                print(f"Ошибка чтения кадра из потока {current_source + 1}")
                cap.release()
                cap = None
                if window_name is not None:
                    try:
                        cv2.destroyWindow(window_name)
                    except cv2.error:
                        pass  # Игнорируем ошибку, если окно не существует
                    window_name = None
                continue

            # Обновление заголовка окна
            if window_name is not None:
                cv2.setWindowTitle(window_name, f"Video Stream (Source {current_source + 1})")
                cv2.imshow(window_name, frame)

            # Обработка нажатий клавиш
            key = cv2.waitKey(1)
            if key == ord('v'):  # Переключение на следующий поток
                if cap is not None:
                    cap.release()
                    cap = None
                if window_name is not None:
                    try:
                        cv2.destroyWindow(window_name)
                    except cv2.error:
                        pass  # Игнорируем ошибку, если окно не существует
                    window_name = None
                current_source = (current_source + 1) % len(video_sources)
                print(f"Переключено на источник: {current_source + 1}")
                cap = open_stream(current_source)
                if cap is not None:
                    window_name = f"Video Stream (Source {current_source + 1})"
                    cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
                continue
            elif key == 27:  # Esc для выхода
                break

    except KeyboardInterrupt:
        print("Программа прервана пользователем")
    except Exception as e:
        print(f"Произошла ошибка: {e}")
    finally:
        # Освобождение ресурсов
        if cap is not None:
            cap.release()
        if window_name is not None:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass  # Игнорируем ошибку, если окна не существуют

if __name__ == "__main__":
    main()