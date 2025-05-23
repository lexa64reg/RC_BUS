import cv2
import socket
import pickle
import struct

def main():
    # Открытие потока с веб-камеры
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Не удалось открыть веб-камеру")
        return

    # Настройка сокета TCP
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    host_ip = '192.168.15.252'  # Замените на IP-адрес сервера
    port = 9999
    socket_address = (host_ip, port)
    server_socket.bind(socket_address)
    server_socket.listen(5)
    print("Сервер запущен, ожидание подключения клиента...")

    client_socket, addr = server_socket.accept()
    print('Клиент подключён:', addr)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Ошибка чтения кадра")
            break

        # Сжатие и сериализация кадра
        frame = cv2.imencode('.jpg', frame)[1]
        data = pickle.dumps(frame, 0)
        size = len(data)

        # Отправка размера кадра
        client_socket.sendall(struct.pack("!L", size))

        # Отправка данных кадра
        client_socket.sendall(data)

if __name__ == "__main__":
    main()
