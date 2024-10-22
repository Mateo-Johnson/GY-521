import pygame
import serial
import time

pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Gyro-Controlled Cube")

WHITE = (255, 255, 255)
CUBE_COLOR = (0, 128, 255)

ser = serial.Serial('COM3', 115200)
time.sleep(2)

def draw_cube(x, y):
    pygame.draw.rect(screen, CUBE_COLOR, (x, y, 50, 50))

running = True
x, y = 375, 275
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        data = line.split(",")
        gx = int(data[3].split(":")[1])
        gy = int(data[4].split(":")[1])

        x += gx // 1000
        y += gy // 1000
        x = max(0, min(800 - 50, x))
        y = max(0, min(600 - 50, y))
  
    screen.fill(WHITE)
    draw_cube(x, y)
    pygame.display.flip()
    pygame.time.delay(30)

pygame.quit()
ser.close()
