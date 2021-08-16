import pygame
from network import Network

width = 500
height = 500
win = pygame.display.set_mode((width, height))
pygame.display.set_caption("Client")

clientNumber = 0


class Player():
    def __init__(self, x, y, width, height, color):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.color = color
        self.rect = (x,y,width,height)
        self.vel = 3

    def draw(self, win):
        pygame.draw.rect(win, self.color, self.rect)

    def move(self):
        
        keys = pygame.key.get_pressed()

        if keys[pygame.K_LEFT]:
            self.x = 0 

        if keys[pygame.K_RIGHT]:
            self.x = 200

        if keys[pygame.K_UP]:
            self.y = 0

        if keys[pygame.K_DOWN]:
            self.y = 200
        
        if keys[pygame.K_LEFT] == False and keys[pygame.K_RIGHT] == False and keys[pygame.K_UP] == False and keys[pygame.K_DOWN]==False:
            self.x = 100
            self.y = 100

        self.update()

    def update(self):
        self.rect = (self.x, self.y, self.width, self.height)


def read_pos(str):
    str = str.split(",")
    return int(str[0]), int(str[1])


def make_pos(tup):
    return str(tup[0]) + "," + str(tup[1])


def redrawWindow(win,player):
    win.fill((255,255,255))
    player.draw(win)
    # player2.draw(win)
    pygame.display.update()


def main():
    run = True
    n = Network()
    # startPos = read_pos(n.getPos())
    # p = Player(startPos[0],startPos[1],100,100,(0,255,0))
    p = Player(0,0,100,100,(255,0,0))
    clock = pygame.time.Clock()

    while run:
        clock.tick(60)
        n.send(make_pos((p.x, p.y)))
        p.update()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
                pygame.quit()

        p.move()
        redrawWindow(win, p)

main()