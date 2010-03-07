from Box2D import *
import contextlib
import math
import pyglet
from pyglet.gl import *

class BodyData(object):
    def __init__(self, actor, body):
        assert isinstance(actor, Actor)
        assert isinstance(body, b2Body)
        self.actor = actor
        self.actor.bodies.add(self)
        self.body = body
        self.body.userData = self

    @property
    def joints(self):
        joints = []
        if self.body is not None:
            joint_edge = self.body.GetJointList()
            while joint_edge is not None:
                joint_data = joint_edge.joint.userData
                assert isinstance(joint_data, JointData)
                joints.append(joint_data)
                joint_edge = joint_edge.next
        return joints

    @property
    def shapes(self):
        shapes = []
        if self.body is not None:
            for shape in self.body.GetShapeList():
                shape_data = shape.userData
                assert isinstance(shape_data, ShapeData)
                shapes.append(shape_data)
        return shapes

    def delete(self):
        for joint_data in self.joints:
            joint_data.delete()
        for shape_data in self.shapes:
            shape_data.delete()
        if self.body is not None:
            self.body.userData = None
            self.body.GetWorld().DestroyBody(self.body)
            self.body = None
        if self.actor is not None:
            self.actor.bodies.remove(self)
            self.actor = None

class ShapeData(object):
    def __init__(self, actor, shape):
        assert isinstance(actor, Actor)
        assert isinstance(shape, b2Shape)
        self.actor = actor
        self.actor.shapes.add(self)
        self.shape = shape
        self.shape.userData = self

    def delete(self):
        if self.shape is not None:
            self.shape.userData = None
            self.shape.GetBody().DestroyShape(self.shape)
            self.shape = None
        if self.actor is not None:
            self.actor.shapes.remove(self)
            self.actor = None

class JointData(object):
    def __init__(self, actor, joint):
        assert isinstance(actor, Actor)
        assert isinstance(joint, b2Joint)
        self.actor = actor
        self.actor.joints.add(self)
        self.joint = joint
        self.joint.userData = self

    def delete(self):
        if self.joint is not None:
            self.joint.userData = None
            self.joint.GetBody1().GetWorld().DestroyJoint(self.joint)
            self.joint = None
        if self.actor is not None:
            self.actor.joints.remove(self)
            self.actor = None

class Actor(object):
    def __init__(self, game_engine):
        assert isinstance(game_engine, GameEngine)
        self.game_engine = game_engine
        self.game_engine.actors.add(self)
        self.group_index = game_engine.generate_group_index()
        self.bodies = set()
        self.shapes = set()
        self.joints = set()

    def delete(self):
        for joint_data in list(self.joints):
            joint_data.delete()
        assert not self.joints
        for shape_data in list(self.shapes):
            shape_data.delete()
        assert not self.shapes
        for body_data in list(self.bodies):
            body_data.delete()
        assert not self.bodies
        if self.game_engine is not None:
            self.game_engine.actors.remove(self)
            self.game_engine = None

    @property
    def first_body_position(self):
        for body_data in self.bodies:
            return body_data.body.position.tuple()
        return 0.0, 0.0

    def create_body(self, position=(0.0, 0.0), angle=0.0,
                    linear_velocity=(0.0, 0.0), angular_velocity=0.0,
                    angular_damping=0.0):
        body_def = b2BodyDef()
        body_def.position = position
        body_def.angle = angle
        body_def.angularDamping = angular_damping
        body = self.game_engine.world.CreateBody(body_def)
        body.SetLinearVelocity(linear_velocity)
        body.SetAngularVelocity(angular_velocity)
        return BodyData(self, body)

    def create_circle_shape(self, body_data, local_position=(0.0, 0.0),
                            radius=1.0, density=0.0, friction=0.2,
                            restitution=0.0, group_index=0):
        assert isinstance(body_data, BodyData)
        assert body_data.body is not None
        assert body_data.actor is self
        shape_def = b2CircleDef()
        shape_def.localPosition = local_position
        shape_def.radius = radius
        shape_def.density = density
        shape_def.friction = friction
        shape_def.restitution = restitution
        shape_def.filter.groupIndex = group_index
        shape = body_data.body.CreateShape(shape_def)
        body_data.body.SetMassFromShapes()
        return ShapeData(self, shape)

    def create_polygon_shape(self, body_data, vertices=None, density=0.0,
                             friction=0.2, restitution=0.0, group_index=0):
        assert isinstance(body_data, BodyData)
        assert body_data.body is not None
        assert body_data.actor is self
        if vertices is None:
            vertices = [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)]
        shape_def = b2PolygonDef()
        shape_def.vertices = vertices
        shape_def.density = density
        shape_def.friction = friction
        shape_def.restitution = restitution
        shape_def.filter.groupIndex = group_index
        shape = body_data.body.CreateShape(shape_def)
        body_data.body.SetMassFromShapes()
        return ShapeData(self, shape)

    def create_box_shape(self, body_data, half_width=1.0, half_height=1.0,
                         local_position=(0.0, 0.0), angle=0.0, density=0.0,
                         friction=0.2, restitution=0.0, group_index=0):
        assert isinstance(body_data, BodyData)
        assert body_data.body is not None
        assert body_data.actor is self
        shape_def = b2PolygonDef()
        shape_def.SetAsBox(half_width, half_height, local_position, angle)
        shape_def.density = density
        shape_def.friction = friction
        shape_def.restitution = restitution
        shape_def.filter.groupIndex = group_index
        shape = body_data.body.CreateShape(shape_def)
        body_data.body.SetMassFromShapes()
        return ShapeData(self, shape)

    def create_revolute_joint(self, body_data_1, body_data_2, anchor):
        joint_def = b2RevoluteJointDef()
        joint_def.Initialize(body_data_1.body, body_data_2.body, anchor)
        joint = self.game_engine.world.CreateJoint(joint_def)
        return JointData(self, joint)

    def create_distance_joint(self, body_data_1, body_data_2, anchor_1,
                              anchor_2):
        joint_def = b2DistanceJointDef()
        joint_def.Initialize(body_data_1.body, body_data_2.body, anchor_1,
                             anchor_2)
        joint = self.game_engine.world.CreateJoint(joint_def)
        return JointData(self, joint)

    def draw(self):
        pass

class TestPlatform(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0), angle=0.0):
        super(TestPlatform, self).__init__(game_engine)
        body_data = self.create_body(position=position)
        self.create_box_shape(body_data, half_width=5.0, half_height=0.1,
                              angle=angle)

class TestVehicle(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0)):
        super(TestVehicle, self).__init__(game_engine)
        position = b2Vec2(position[0], position[1])
        body_data_1 = self.create_body(position=position)
        self.create_box_shape(body_data_1, half_width=1.0, half_height=0.5,
                              density=1000.0, group_index=-self.group_index)
        body_data_2 = self.create_body(position=(position + b2Vec2(-0.7, -0.4)))
        self.create_circle_shape(body_data_2, radius=0.5, density=100.0,
                                 group_index=-self.group_index)
        self.create_revolute_joint(body_data_1, body_data_2,
                                   body_data_2.body.GetWorldCenter())
        body_data_3 = self.create_body(position=(position + b2Vec2(0.7, -0.4)))
        self.create_circle_shape(body_data_3, radius=0.5, density=100.0,
                                 group_index=-self.group_index)
        self.create_revolute_joint(body_data_1, body_data_3,
                                   body_data_3.body.GetWorldCenter())

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

class Camera(Actor):
    def __init__(self, game_engine):
        super(Camera, self).__init__(game_engine)
        self.scale = 50.0
        self.position = 0.0, 0.0

    @contextlib.contextmanager
    def transform(self, width, height):
        glPushMatrix()
        glTranslatef(float(width // 2), float(height // 2), 0.0)
        glScalef(self.scale, self.scale, self.scale)
        x, y = self.position
        glTranslatef(-x, -y, 0.0)
        yield None
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
        self._init_world()
        self.player_actor = None
        self.debug_draw = MyDebugDraw()
        self.world.SetDebugDraw(self.debug_draw)
        self.destruction_listener = MyDestructionListener()
        self.world.SetDestructionListener(self.destruction_listener)        
        self.scheduler = Scheduler()
        self.actors = set()
        self.controllers = set()
        self.camera = Camera(self)
        self._init_test_actors()

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

    def _init_world(self):
        aabb = b2AABB()
        aabb.lowerBound = -1000.0, -1000.0
        aabb.upperBound = 1000.0, 1000.0
        gravity = 0.0, -10.0
        self.world = b2World(aabb, gravity, True)

    def _init_test_actors(self):
        TestPlatform(self, angle=-0.2)
        TestPlatform(self, position=(9.0, -2.0), angle=0.1)
        self.player_actor = TestVehicle(self, position=(-3.0, 3.0))

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
        if self.player_actor is not None:
            self.camera.position = self.player_actor.first_body_position
        with self.camera.transform(width, height):
            for actor in self.actors:
                actor.draw()
            if self.debug_draw is not None:
                self.debug_draw.draw()

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
        self.compiling = False

    def delete(self):
        assert not self.compiling
        if self.circle_display_list is not None:
            self.circle_display_list.delete()
            self.circle_display_list = None
        if self.display_list:
            glDeleteLists(self.display_list, 1)
            self.display_list = 0

    @contextlib.contextmanager
    def compile(self):
        assert not self.compiling
        self.compiling = True
        glNewList(self.display_list, GL_COMPILE)
        glPushAttrib(GL_ALL_ATTRIB_BITS)
        glPushMatrix()
        yield None
        glPopMatrix()
        glPopAttrib()
        glEndList()
        self.compiling = False

    def draw(self):
        assert not self.compiling
        glCallList(self.display_list)

    def DrawCircle(self, center, radius, color):
        assert self.compiling
        glColor3f(color.r, color.g, color.b)
        self.circle_display_list.draw(center.tuple(), radius, GL_LINE_LOOP)

    def DrawSegment(self, p1, p2, color):
        assert self.compiling
        glColor3f(color.r, color.g, color.b)
        glBegin(GL_LINES)
        glVertex2f(p1.x, p1.y)
        glVertex2f(p2.x, p2.y)
        glEnd()

    def DrawXForm(self, xf):
        assert self.compiling
        glPopMatrix()
        glPushMatrix()
        glTranslatef(xf.position.x, xf.position.y, 0.0)
        glRotatef(xf.R.GetAngle() * 180.0 / math.pi, 0.0, 0.0, 1.0)

    def DrawSolidCircle(self, center, radius, axis, color):
        assert self.compiling
        glColor3f(color.r, color.g, color.b)
        self.circle_display_list.draw(center.tuple(), radius, GL_POLYGON)
        self.DrawSegment(center, center + radius * axis, self.axis_color)

    def DrawPolygon(self, vertices, vertexCount, color):
        assert self.compiling
        glColor3f(color.r, color.g, color.b)
        glBegin(GL_LINE_LOOP)
        for i in xrange(vertexCount):
            glVertex2f(*vertices[i])
        glEnd()

    def DrawSolidPolygon(self, vertices, vertexCount, color):
        assert self.compiling
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
        self.game_engine.delete()
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
    config = pyglet.gl.Config(double_buffer=True, sample_buffers=1, samples=4,
                              depth_size=8)
    window = MyWindow(config=config)
    pyglet.app.run()

if __name__ == '__main__':
    main()