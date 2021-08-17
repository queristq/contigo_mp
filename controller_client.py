import pygame
from network import Network

width = 800
height = 500
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("Client")

clientNumber = 0

class Player():
    def __init__(self, x_wheel, y_wheel, x_pantilt, y_pantilt, width, height):
        self.x_wheel = x_wheel
        self.y_wheel = y_wheel
        self.x_pantilt = x_pantilt
        self.y_pantilt = y_pantilt
        self.width = width
        self.height = height
        self.color_wheel = (255,0,0)
        self.color_pantilt = (0,255,0)
        self.rect_wheel = (x_wheel,y_wheel,width,height)
        self.rect_pantilt = (x_pantilt,y_pantilt,width,height)
        self.vel = 3

    def draw(self, win):
        pygame.draw.rect(win, self.color_wheel, self.rect_wheel)
        pygame.draw.rect(win, self.color_pantilt, self.rect_pantilt)

    def move(self):
        
        keys = pygame.key.get_pressed()

        if keys[pygame.K_LEFT]:
            self.x_wheel = 0 

        if keys[pygame.K_RIGHT]:
            self.x_wheel = 200

        if keys[pygame.K_UP]:
            self.y_wheel = 0

        if keys[pygame.K_DOWN]:
            self.y_wheel = 200
        
        if keys[pygame.K_LEFT] == False and keys[pygame.K_RIGHT] == False and keys[pygame.K_UP] == False and keys[pygame.K_DOWN]==False:
            self.x_wheel = 100
            self.y_wheel = 100

        if keys[pygame.K_a]:
            self.x_pantilt = 0 

        if keys[pygame.K_d]:
            self.x_pantilt = 200

        if keys[pygame.K_w]:
            self.y_pantilt = 0

        if keys[pygame.K_s]:
            self.y_pantilt = 200
        
        if keys[pygame.K_a] == False and keys[pygame.K_d] == False and keys[pygame.K_w] == False and keys[pygame.K_s]==False:
            self.x_pantilt = 100
            self.y_pantilt = 100

        self.update()

    def update(self):
        self.rect_wheel = (self.x_wheel, self.y_wheel, self.width, self.height)
        self.rect_pantilt = (self.x_pantilt + 400 , self.y_pantilt, self.width, self.height)

# def read_pos(str):
#     str = str.split(",")
#     return int(str[0]), int(str[1])

def make_pos(tup):
    return str(tup[0]) + "," + str(tup[1])+ "," + str(tup[2]) + "," + str(tup[3])

def redrawWindow(win,player):
    win.fill((255,255,255))
    player.draw(win)
    pygame.display.update()

def main():
    run = True
    n = Network()
    p = Player(0,0,0,0,100,100)
    clock = pygame.time.Clock()

    while run:
        clock.tick(60)
        n.send(make_pos((p.x_wheel, p.y_wheel, p.x_pantilt, p.y_pantilt)))
        p.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()

        p.move()
        redrawWindow(win, p)

main()