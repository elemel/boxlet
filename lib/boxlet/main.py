from boxlet.game_engine import GameEngine

import pyglet
from pyglet.gl import *

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        self.time = 0.0
        self.dt = 1.0 / 60.0
        self.game_engine = GameEngine()
        self.clock_display = pyglet.clock.ClockDisplay()
        pyglet.clock.schedule_interval(self.step, self.dt)

    def close(self):
        pyglet.clock.unschedule(self.step)
        self.game_engine.delete()
        super(MyWindow, self).close()

    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.game_engine.draw(self.width, self.height)
        self.clock_display.draw()

    def step(self, dt):
        self.time += dt
        while self.game_engine.time + self.dt <= self.time:
            self.game_engine.step(self.dt)

    def on_key_press(self, key, modifiers):
        if key == pyglet.window.key.ESCAPE:
            self.close()
        else:
            self.game_engine.on_key_press(key, modifiers)

    def on_key_release(self, key, modifiers):
        self.game_engine.on_key_release(key, modifiers)

def main():
    config = pyglet.gl.Config(double_buffer=True, sample_buffers=1, samples=4,
                              depth_size=8)
    window = MyWindow(fullscreen=True, config=config)
    pyglet.app.run()

if __name__ == '__main__':
    main()
