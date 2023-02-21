import pygame
import random

# red then green no stop
#random.seed(10)
# stop with red
#random.seed(1)

from pydrivingsim import VirtualObject, World

class BarricadeSprite(pygame.sprite.Sprite):
    def __init__(self, barricade):
        super().__init__()
        img = "imgs/barricade.png"
        #sprite = pygame.image.load(img).convert_alpha()
        sprite = pygame.image.load(img)
        #sprite.set_alpha(512)
        self.w, self.h = sprite.get_size()
        scale = (World().scaling_factor * 1.2) / self.w
        self.image_fix = []
        self.image_fix.append(pygame.transform.smoothscale(sprite, (int(self.w * scale), int(self.h * scale))))
        
        self.image = self.image_fix[0]
        self.rect = self.image_fix[0].get_rect()
        self.size = self.image_fix[0].get_size()
        self.scale_w,self.scale_h  = self.image_fix[0].get_size()
        self.scale_w = self.scale_w * World().scaling_factor*4
        self.scale_h = self.scale_h * World().scaling_factor*4
        # = self.h * scale
        self.barricade = barricade

    def update(self) -> None:
        # print("Obstacle size")
        # print(self.size)
        self.rect.center = [
            self.size[0] / 2 + self.barricade.pos[0] * World().scaling_factor - World().get_world_pos()[0],
            self.barricade.pos[1] * World().scaling_factor - World().get_world_pos()[1]
        ]
        print("rect centre")
        print(self.rect.center)
        self.image = self.image_fix[self.barricade.state]


class Barricade(VirtualObject):
    __metadata = {
        "dt": 0.1
    }
    def __init__( self ):
        super().__init__(self.__metadata["dt"])
        # Sprite
        self.barricade = BarricadeSprite(self)
        self.group = pygame.sprite.Group()
        self.group.add(self.barricade)
        self.w1=0
        self.h1=0

        self.pos = (3,0)
        self.time_phases = (8,3,8)
        self.time_past_switch = 0

        #self.clock = None
        self.state = 0
        #self.reset()

    def set_pos(self, point: tuple):
        self.pos = point

    def set_time_phases(self, time_phases: tuple):
        self.time_phases = time_phases

    def reset( self ):
        #Initial condition of the vehicle
        self.state = 0
        self.time_past_switch = random.random()*self.time_phases[self.state]

    def object_freq_compute(self):
        self.time_past_switch += self.__metadata["dt"]
        if self.time_past_switch >= self.time_phases[self.state]:
            self.state = 0
            self.time_past_switch = 0

    # def step(self):
    #     self.num_of_step += 1
    #     if self.num_of_step >= self.sim_call_freq:
    #         self.time_past_switch += self.__metadata["dt"]
    #         if self.time_past_switch >= self.time_phases[self.state]:
    #             self.state = divmod(self.state + 1,3)[1]
    #             self.time_past_switch = 0
    #         self.num_of_step = 0

    def getb_size(self,w, h):
        #self.w1,self.h1 = image.get_size()/World().scaling_factor * 1.2

        #self.w1,self.h1 = tuple(ti*w/World().scaling_factor * 1.2 for ti in image.get_size())
        self.w1,self.h1 = w,h

        

    def render( self ):
        self.barricade.update()
        self.getb_size(self.barricade.scale_w,self.barricade.scale_h)
        self.group.draw(World().screen)


    def get_state(self):
        return self.state