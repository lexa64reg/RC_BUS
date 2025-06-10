import cv2
import time
from decouple import config, Csv
import threading
import routeros_api
api = None
connection = None

# Параметры подключения к роутеру
HOST = config('MIKROTIK_IP')  
USERNAME = config('MIKROTIK_LOGIN')  
PASSWORD = config('MIKROTIK_PASS')        
PORT = config('MIKROTIK_PORT_API')    

# Инициализация подключения к MikroTik
def init_api():
    global api, connection
    try:
        connection = routeros_api.RouterOsApiPool(
            host=HOST,
            username=USERNAME,
            password=PASSWORD,
            port=PORT,
            use_ssl=False,  # Установите True, если используете API-SSL
            plaintext_login=True
        )
        api = connection.get_api()
    except Exception as e:
        print(f"Ошибка инициализации соединения с роутером: {str(e)}")
        connection = None
        api = None
#-------------------------------------------
voltage = "N/A"
voltage_lock = threading.Lock()
run = True
init_api()
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

def get_voltage():
    global api
    try:
        if api is None:
            print("API не инициализировано")
            init_api()
            return None
        # Получаем данные из /system/health
        health = api.get_resource('/system/health')
        health_data = health.get()

        # Ищем значение напряжения
        for item in health_data:
            if 'name' in item and item['name'] == 'voltage':
                voltage_value = item.get('value', 'N/A')
                #print(f"Напряжение: {voltage_value}V")
                #time.sleep(0.01)
                break
        return voltage_value

    except Exception as e:
        print(f"Ошибка получения данных с роутера: {str(e)}")
        return None

def voltage_thread():
    global voltage, run
    while run:
        new_voltage = get_voltage()
        with voltage_lock:
            if new_voltage is not None:
                voltage = new_voltage
        time.sleep(2)

def open_stream(source_index):
    """Открывает видеопоток по указанному индексу."""
    try:
        cap = cv2.VideoCapture(video_sources[source_index])
        if not cap.isOpened():
            print(f"Ошибка открытия потока {source_index + 1}")
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
    global run
    # Попытка открыть начальный поток
    cap = open_stream(current_source)
    window_name = None
    
    # Создание окна, если поток открыт
    if cap is not None:
        window_name = f"Video Stream (Source {current_source + 1})"
        cv2.namedWindow(window_name, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:

        voltage_thread_instance = threading.Thread(target=voltage_thread, daemon=True)
        voltage_thread_instance.start()
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


            text = f"{voltage} V"
            font = cv2.FONT_HERSHEY_DUPLEX
            font_scale = 1.0
            color = (255, 255, 255)  # Белый цвет текста
            thickness = 2
            position = (10, 30)  # Положение текста (x=10, y=30 от верхнего левого угла)
            height, width = frame.shape[:2]

            # Вычисляем размер текста
            (text_width, text_height), baseline = cv2.getTextSize(text, font, font_scale, thickness)

            # Рассчитываем координаты для центрирования по горизонтали и размещения внизу
            x = (width - text_width) // 2  # Центр по горизонтали
            y = height - 10 - baseline  # 10 пикселей отступа от нижнего края

            # Наложение текста на кадр
            cv2.putText(frame, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)
            #cv2.putText(frame, text, position, font, font_scale, color, thickness, cv2.LINE_AA)

            # Обновление заголовка окна
            if window_name is not None:
                cv2.setWindowTitle(window_name, f"Video Stream (Source {current_source + 1})")
                cv2.imshow(window_name, frame)

            # Обработка нажатий клавиш
            key = cv2.waitKey(1)
            if key == ord('v') or key == ord('V'):  # Переключение на следующий поток
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
        if connection is not None:
            connection.disconnect()
        if cap is not None:
            cap.release()
        if window_name is not None:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass  # Игнорируем ошибку, если окна не существуют
        run = False

if __name__ == "__main__":
    main()