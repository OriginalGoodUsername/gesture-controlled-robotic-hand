"""Alternate open and closed poses over serial without camera input."""
import serial
import time

OPEN   = [90, 0,   0,   180, 180, 90]
CLOSED = [90, 180, 180, 0,   0,   0]

s = serial.Serial("COM5", 9600)
time.sleep(2)  # Arduino resets when the port opens; wait for it to boot
print("port open, sending sweeps...")

for i in range(6):
    pose = CLOSED if i % 2 else OPEN
    msg = ",".join(map(str, pose)) + "\n"
    s.write(msg.encode())
    print("sent:", msg.strip())
    time.sleep(1.2)

s.write((",".join(map(str, OPEN)) + "\n").encode())
print("done, returned to open")
s.close()
