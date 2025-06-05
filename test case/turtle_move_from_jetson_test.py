import socket

def send_command(x, z, ip='192.168.50.129', port=5025):
    message = f"{x},{z}"
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(message.encode(), (ip, port))
    print(f"Sent command: {message} to {ip}:{port}")

if __name__ == "__main__":
    # Example: move forward at 0.2 m/s and rotate at 0.5 rad/s
    send_command(1.0, 0.4)

    # Optionally wait and stop the robot
    import time
    time.sleep(2)


