import cv2
import socket
import pickle
import struct

def main():
    # Настройка сокета TCP
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host_ip = '192.168.15.252'  # Замените на IP-адрес сервера
    port = 9999
    client_socket.connect((host_ip, port))
    print("Клиент запущен, ожидание данных...")

    try:
        while True:
            # Получение размера кадра
            packet = client_socket.recv(4)
            if not packet: break
            size = struct.unpack("!L", packet)[0]

            # Получение данных кадра
            data = b""
            while len(data) < size:
                packet = client_socket.recv(size - len(data))
                if not packet: break
                data += packet

            frame = pickle.loads(data)
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)

            # Отображение кадра на весь экран
            #cv2.namedWindow("Received Video", cv2.WND_PROP_FULLSCREEN)
            #cv2.setWindowProperty("Received Video", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow("Received Video", frame)

            key = cv2.waitKey(1)
            if key == 27:  # Esc - выход
                break

    except Exception as e:
        print(f"Произошла ошибка: {e}")

    finally:
        # Освобождение ресурсов
        client_socket.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
