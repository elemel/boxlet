import boxlet.actors
import boxlet.controllers
from boxlet.debug_draw import *
from boxlet.lighting import *
from boxlet.utils import *

from Box2D import *
import contextlib
from ctypes import c_float
import math
import pyglet
from pyglet.gl import *
import random
import shader

class Camera(object):
    def __init__(self, resolution=(1, 1), position=(0.0, 0.0), scale=50.0):
        self.resolution = resolution
        self.position = position
        self.scale = scale

    def __enter__(self):
        glPushMatrix()
        width, height = self.resolution
        glTranslatef(float(width // 2), float(height // 2), 0.0)
        glScalef(self.scale, self.scale, self.scale)
        x, y = self.position
        glTranslatef(-x, -y, 0.0)

    def __exit__(self, *args):
        glPopMatrix()

class MyDestructionListener(b2DestructionListener):
    def __init__(self):
        super(MyDestructionListener, self).__init__()

    def SayGoodbye(self, shape_or_joint):
        assert False

class GameEngine(object):
    def __init__(self):
        self.time = 0.0
        self._next_group_index = 1
        self._init_lighting()
        self._init_world()
        self.player_actor = None
        self.debug_draw = None # MyDebugDraw()
        self.world.SetDebugDraw(self.debug_draw)
        self.destruction_listener = MyDestructionListener()
        self.world.SetDestructionListener(self.destruction_listener)        
        self.scheduler = Scheduler()
        self.actors = set()
        self.controllers = set()
        water_frag = pyglet.resource.file('resources/shaders/water.frag').read()
        self.water_shader = MyShader(frag=[water_frag])
        self.camera = Camera()
        self._init_test_actors()
        self.background_image = pyglet.resource.image('resources/images/cave.jpg')
        self.background_image.anchor_x = self.background_image.width // 2
        self.background_image.anchor_y = self.background_image.height // 2
        self.background_color = 1.0, 0.8, 0.6
        self.environment_image = pyglet.resource.image('resources/images/cave-lighting.jpg')
        self.environment_image_data = self.environment_image.image_data
        self.environment_scale = 1.5
        self.mouse_position = 0, 0

    def delete(self):
        for actor in list(self.actors):
            actor.delete()
        assert not self.actors
        if self.debug_draw is not None:
            self.debug_draw.delete()
            self.debug_draw = None

    def generate_group_index(self):
        group_index = self._next_group_index
        self._next_group_index += 1
        return group_index

    def _init_lighting(self):
        glEnable(GL_NORMALIZE)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        self.lighting_manager = LightingManager()
        light_1 = DirectionalLight(self.lighting_manager,
                                   color=(1.0, 1.0, 1.0),
                                   direction=(1.0, -1.0, -1.0))
        light_2 = DirectionalLight(self.lighting_manager,
                                   color=(0.5, 0.5, 0.5),
                                   direction=(-1.0, -1.0, -1.0))
        light_3 = DirectionalLight(self.lighting_manager,
                                   color=(0.1, 0.1, 0.1),
                                   direction=(-1.0, 1.0, -1.0))
        light_4 = DirectionalLight(self.lighting_manager,
                                   color=(0.2, 0.2, 0.2),
                                   direction=(1.0, 1.0, -1.0))
        self.environment_lights = [(1.0, light_1), (0.5, light_2),
                                   (0.1, light_3), (0.2, light_4)]
        self.mouse_light = SpotLight(self.lighting_manager)

    def _init_world(self):
        aabb = b2AABB()
        aabb.lowerBound = -1000.0, -1000.0
        aabb.upperBound = 1000.0, 1000.0
        gravity = 0.0, -10.0
        self.world = b2World(aabb, gravity, True)

    def _init_test_actors(self):
        boxlet.actors.TestPlatformActor(self, angle=-0.2)
        boxlet.actors.TestPlatformActor(self, position=(9.0, -2.0), angle=0.1)
        boxlet.actors.WaterActor(self, vertices=get_box_vertices(half_width=10.0))
        self.player_actor = boxlet.actors.TestVehicleActor(self, position=(-3.0, 3.0))

    def step(self, dt):
        self.time += dt
        self.scheduler.dispatch(self.time)
        self._step_controllers(dt)
        if self.debug_draw is not None:
            with self.debug_draw.compile():
                self.world.Step(dt, 10, 10)
        else:
            self.world.Step(dt, 10, 10)

    def _step_controllers(self, dt):
        for controller in list(self.controllers):
            if controller in self.controllers:
                controller.step(dt)

    def draw(self, width, height):
        self.camera.resolution = width, height
        if self.player_actor is not None:
            self.camera.position = self.player_actor.first_body_position
        mouse_x, mouse_y = self.mouse_position
        self.mouse_light.position = mouse_x, mouse_y, 100.0
        self._update_environment_lighting()
        glPushAttrib(GL_ALL_ATTRIB_BITS)
        glPushMatrix()
        glColor3f(*self.background_color)
        glScalef(1.2, 1.2, 1.2)
        glTranslatef(-5.0 * self.camera.position[0], -5.0 * self.camera.position[1], 0.0)
        self.background_image.blit(width // 2, height // 2)
        glPopMatrix()
        glPopAttrib()
        with self.camera:
            glPushAttrib(GL_ALL_ATTRIB_BITS)
            with self.lighting_manager:
                for actor in self.actors:
                    actor.draw()
            glPopAttrib()
            if self.debug_draw is not None:
                self.debug_draw.draw()

    def _update_environment_lighting(self):
        for light_scale, light in self.environment_lights:
            x, y, z = normalize(light.direction)
            x = self.environment_image_data.width // 2 - int(float(self.environment_image_data.width // 4) * x)
            y = self.environment_image_data.height // 2 - int(float(self.environment_image_data.height // 4) * y)
            background_r, background_g, background_b = self.background_color
            sample_r, sample_g, sample_b = sample_image(self.environment_image_data, x, y)[:3]
            light_r = light_scale * self.environment_scale * background_r * sample_r
            light_g = light_scale * self.environment_scale * background_g * sample_g
            light_b = light_scale * self.environment_scale * background_b * sample_b
            light.color = light_r, light_g, light_b
            
    def on_key_press(self, key, modifiers):
        if key == pyglet.window.key.C:
            self.background_color = tuple(random.uniform(0.5, 1.0) for _ in xrange(3))
        elif self.player_actor is not None:
            self.player_actor.on_key_press(key, modifiers)

    def on_key_release(self, key, modifiers):
        if self.player_actor is not None:
            self.player_actor.on_key_release(key, modifiers)

    def on_mouse_drag(self, x, y, dx, dy, buttons, modifiers):
        self.mouse_position = x, y

    def on_mouse_enter(self, x, y):
        self.mouse_position = x, y

    def on_mouse_leave(self, x, y):
        self.mouse_position = x, y

    def on_mouse_motion(self, x, y, dx, dy):
        self.mouse_position = x, y

    def on_mouse_press(self, x, y, button, modifiers):
        self.mouse_position = x, y

    def on_mouse_release(self, x, y, button, modifiers):
        self.mouse_position = x, y

    def on_mouse_scroll(self, x, y, scroll_x, scroll_y):
        self.mouse_position = x, y

class Scheduler(object):
    def __init__(self):
        self.calls = []
        self.tiebreaker = 0

    def schedule(self, time, func, *arg, **kwargs):
        assert hasattr(func, '__call__')
        call = [time, self.tiebreaker, func, arg, kwargs]
        self.tiebreaker += 1
        heappush(self.calls, call)
        return call

    def unschedule(self, call):
        del call[2:]

    def dispatch(self, time):
        while self.calls and self.calls[0][0] <= time:
            call = heappop(self.calls)
            if len(call) == 5:
                func, args, kwargs = call[2:]
                func(*args, **kwargs)            

class MyShader(shader.Shader):
    def __enter__(self):
        self.bind()
        return self

    def __exit__(self, *args):
        self.unbind()
