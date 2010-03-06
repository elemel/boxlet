from Box2D import *
import contextlib
import math
import pyglet
from pyglet.gl import *

class Actor(object):
    def __init__(self, game_engine):
        self.game_engine = game_engine
        self.game_engine.actors.add(self)

    def delete(self):
        if self.game_engine is not None:
            self.game_engine.actors.remove(self)
            self.game_engine = None

    def draw(self):
        pass

class Controller(object):
    def __init__(self, game_engine):
        self.game_engine = game_engine
        self.game_engine.controllers.add(self)

    def delete(self):
        if self.game_engine is not None:
            self.game_engine.controllers.remove(self)
            self.game_engine = None

    def step(self, dt):
        pass

class Camera(object):
    def __init__(self):
        self.scale = 50.0
        self.position = 0.0, 0.0

class GameEngine(object):
    def __init__(self):
        self.time = 0.0
        self._init_world()
        self.debug_draw = MyDebugDraw()
        self.world.SetDebugDraw(self.debug_draw)
        self.scheduler = Scheduler()
        self.actors = set()
        self.controllers = set()
        self.camera = Camera()

    def delete(self):
        if self.debug_draw is not None:
            self.debug_draw.delete()
            self.debug_draw = None

    def _init_world(self):
        aabb = b2AABB()
        aabb.lowerBound = -1000.0, -1000.0
        aabb.upperBound = 1000.0, 1000.0
        gravity = 0.0, -10.0
        self.world = b2World(aabb, gravity, True)
        
        body_def = b2BodyDef()
        body_def.angle = math.pi / 4.0
        self.test_body = self.world.CreateBody(body_def)
        shape_def = b2CircleDef()
        shape_def.localPosition = 0.0, 0.0
        shape_def.radius = 1.0
        self.test_shape = self.test_body.CreateShape(shape_def)

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
        glPushMatrix()
        glTranslatef(float(width // 2), float(height // 2), 0.0)
        glScalef(self.camera.scale, self.camera.scale, self.camera.scale)
        for actor in self.actors:
            actor.draw()
        if self.debug_draw is not None:
            self.debug_draw.draw()
        glPopMatrix()

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

class CircleDisplayList(object):
    def __init__(self, vertex_count=64):
        self.display_list = glGenLists(1)
        glNewList(self.display_list, GL_COMPILE)
        for i in xrange(vertex_count):
            angle = 2.0 * math.pi * float(i) / float(vertex_count)
            glVertex2f(math.cos(angle), math.sin(angle))
        glEndList()

    def delete(self):
        if self.display_list:
            glDeleteLists(self.display_list, 1)
            self.display_list = 0

    def draw(self, center=(0.0, 0.0), radius=1.0, mode=GL_LINE_LOOP):
        glPushMatrix()
        glTranslatef(center[0], center[1], 0.0)
        glScalef(radius, radius, 1.0)
        glBegin(mode)
        glCallList(self.display_list)
        glEnd()
        glPopMatrix()

class MyDebugDraw(b2DebugDraw):
    def __init__(self, flags=None):
        super(MyDebugDraw, self).__init__()
        if flags is None:
            flags = (self.e_aabbBit | self.e_centerOfMassBit |
                     self.e_controllerBit | self.e_coreShapeBit |
                     self.e_jointBit | self.e_obbBit | self.e_pairBit |
                     self.e_shapeBit)
        self.SetFlags(flags)
        self.display_list = glGenLists(1)
        self.circle_display_list = CircleDisplayList()
        self.axis_color = b2Color(1.0, 0.0, 1.0)

    def delete(self):
        if self.circle_display_list is not None:
            self.circle_display_list.delete()
            self.circle_display_list = None
        if self.display_list:
            glDeleteLists(self.display_list, 1)
            self.display_list = 0

    @contextlib.contextmanager
    def compile(self):
        glNewList(self.display_list, GL_COMPILE)
        glPushAttrib(GL_ALL_ATTRIB_BITS)
        glPushMatrix()
        yield None
        glPopMatrix()
        glPopAttrib()
        glEndList()

    def draw(self):
        glCallList(self.display_list)

    def DrawCircle(self, center, radius, color):
        glColor3f(color.r, color.g, color.b)
        self.circle_display_list.draw(center.tuple(), radius, GL_LINE_LOOP)

    def DrawSegment(self, p1, p2, color):
        glColor3f(color.r, color.g, color.b)
        glBegin(GL_LINES)
        glVertex2f(p1.x, p1.y)
        glVertex2f(p2.x, p2.y)
        glEnd()

    def DrawXForm(self, xf):
        glPopMatrix()
        glPushMatrix()
        glTranslatef(xf.position.x, xf.position.y, 0.0)
        glRotatef(xf.R.GetAngle() * 180.0 / math.pi, 0.0, 0.0, 1.0)

    def DrawSolidCircle(self, center, radius, axis, color):
        glColor3f(color.r, color.g, color.b)
        self.circle_display_list.draw(center.tuple(), radius, GL_POLYGON)
        self.DrawSegment(center, center + axis, self.axis_color)

    def DrawPolygon(self, vertices, vertexCount, color):
        glColor3f(color.r, color.g, color.b)
        glBegin(GL_LINE_LOOP)
        for i in xrange(vertexCount):
            glVertex2f(*vertices[i])
        glEnd()

    def DrawSolidPolygon(self, vertices, vertexCount, color):
        glColor3f(color.r, color.g, color.b)
        glBegin(GL_POLYGON)
        for i in xrange(vertexCount):
            glVertex2f(*vertices[i])
        glEnd()

class MyWindow(pyglet.window.Window):
    def __init__(self, **kwargs):
        super(MyWindow, self).__init__(**kwargs)
        self.time = 0.0
        self.dt = 1.0 / 60.0
        self.game_engine = GameEngine()
        self.clock_display = pyglet.clock.ClockDisplay()
        pyglet.clock.schedule_interval(self.step, self.dt)

    def close(self):
        pyglet.clock.unschedule(self.step)
        super(MyWindow, self).close()

    def on_draw(self):
        self.clear()
        self.game_engine.draw(self.width, self.height)
        self.clock_display.draw()

    def step(self, dt):
        self.time += dt
        while self.game_engine.time + self.dt <= self.time:
            self.game_engine.step(self.dt)

def main():
    window = MyWindow(fullscreen=True)
    pyglet.app.run()

if __name__ == '__main__':
    main()