import socket, struct, time, keyboard

SERVER = "navio.local"
PORT = 5001

FWD_TIMING = 2
BACK_TIMING = 1
STOP_TIMING = 1.5

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(enable, pwms):
    socket.sendto(struct.pack("!qB14d", int(round(time.time() * 1000)),
                              0b10000000 if enable else 0b00000000, *pwms),
                  (SERVER, PORT))


if __name__ == '__main__':
    while True:
        left = STOP_TIMING
        right = STOP_TIMING

        # there's def a more elegant way to do this
        if keyboard.is_pressed('d'):
            if keyboard.is_pressed('a'):
                if keyboard.is_pressed('w'):
                    left = right = FWD_TIMING
                elif keyboard.is_pressed('s'):
                    left = right = BACK_TIMING
                else:
                    left = right = STOP_TIMING
            else:
                if keyboard.is_pressed('w'):
                    left = STOP_TIMING
                    right = FWD_TIMING
                elif keyboard.is_pressed('s'):
                    left = STOP_TIMING
                    right = BACK_TIMING
                else:
                    left = BACK_TIMING
                    right = FWD_TIMING
        elif keyboard.is_pressed('a'):
            if keyboard.is_pressed('w'):
                left = FWD_TIMING
                right = STOP_TIMING
            elif keyboard.is_pressed('s'):
                left = BACK_TIMING
                right = STOP_TIMING
            else:
                left = FWD_TIMING
                right = BACK_TIMING
        elif keyboard.is_pressed('w'):
            left = right = FWD_TIMING
        elif keyboard.is_pressed('s'):
            left = right = BACK_TIMING

        pwms = (left, right, 2.33 if keyboard.is_pressed("space") else 1.55, -1,
                -1, -1, -1, -1, -1, -1, -1, -1, -1, -1)
        send(keyboard.is_pressed('shift'), pwms)
        time.sleep(0.05)
