import pygame



class KeyboardController:
    def __init__(self):
        self.keys = {pygame.K_w: False, pygame.K_a: False, pygame.K_s: False, pygame.K_d: False, pygame.K_UP: False, pygame.K_DOWN: False}

    def update(self, event):
        self.event = event
        if self.event.type == pygame.KEYDOWN:
            self.keys[self.event.key] = True
        elif self.event.type == pygame.KEYUP:
            self.keys[self.event.key] = False

    def is_pressed(self, key) -> bool:
        return self.keys[key]
