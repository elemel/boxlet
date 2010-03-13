from boxlet.lighting import *

from Box2D import *
import contextlib
from ctypes import c_float
import math
import pyglet
from pyglet.gl import *
import shader

def clamp(x, min_x, max_x):
    return min(max(x, min_x), max_x)

def float_to_ubyte(f):
    return clamp(int(f * 256.0), 0, 255)

def float_to_byte(f):
    return clamp(int(math.floor(f * 128.0)), -128, 127)

def get_box_vertices(half_width=1.0, half_height=1.0):
    return [(-half_width, -half_height), (half_width, -half_height),
            (half_width, half_height), (-half_width, half_height)]

def get_circle_vertices(center=(0.0, 0.0), radius=1.0, angle=0.0, count=8):
    vertices = []
    x, y = center
    for i in xrange(count):
        a = angle + 2.0 * math.pi * float(i) / float(count)
        vertex = x + radius * math.cos(a), y + radius * math.sin(a)
        vertices.append(vertex)
    return vertices

def get_circle_triangles(center=(0.0, 0.0), radius=1.0, angle=0.0, count=8):
    vertices = get_circle_vertices(center, radius, angle, count)
    triangles = []
    for i in xrange(count):
        j = (i + 1) % count
        triangle = center, vertices[i], vertices[j]
        triangles.append(triangle)
    return triangles

def get_polygon_triangles(vertices):
    centroid = get_polygon_centroid(vertices)
    triangles = []
    for i in xrange(len(vertices)):
        j = (i + 1) % len(vertices)
        triangle = centroid, vertices[i], vertices[j]
        triangles.append(triangle)
    return triangles

# TODO: calculate actual centroid
def get_polygon_centroid(vertices):
    total_x = sum(x for x, y in vertices)
    total_y = sum(y for x, y in vertices)
    return total_x / float(len(vertices)), total_y / float(len(vertices))

def get_vertex_normals(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    n1 = 0.0, 0.0, 1.0
    n2 = normalize((x2 - x1, y2 - y1, 0.0))
    n3 = normalize((x3 - x1, y3 - y1, 0.0))
    return [n1, n2, n3]

def get_face_normal(p1, p2, p3):
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x = (x2 + x3) / 2.0
    y = (y2 + y3) / 2.0
    nx, ny = x - x1, y - y1
    nz = abs((nx + ny) / 2.0)
    return normalize((nx, ny, nz))

def normalize(v):
    x, y, z = v
    if x or y or z:
        length = math.sqrt(x * x + y * y + z * z)
        assert length
        return x / length, y / length, z / length
    else:
        return 0.0, 0.0, 0.0

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

class BodyData(object):
    def __init__(self, actor, body):
        assert isinstance(actor, Actor)
        assert isinstance(body, b2Body)
        self.actor = actor
        self.actor.bodies.add(self)
        self.body = body
        self.body.userData = self
        self.vertex_list_dirty = True
        self.vertex_list = None

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
        if self.vertex_list is not None:
            self.vertex_list.delete()
            self.vertex_list = None
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

    def draw(self):
        if self.vertex_list_dirty:
            self.vertex_list = self.create_vertex_list()
            self.vertex_list_dirty = False
        if self.vertex_list is not None:
            glPushMatrix()
            glTranslatef(self.body.position.x, self.body.position.y, 0.0)
            glRotatef(self.body.angle * 180.0 / math.pi, 0.0, 0.0, 1.0)
            self.vertex_list.draw(GL_TRIANGLES)
            glPopMatrix()

    def create_vertex_list(self):
        vertices = []
        normals = []
        colors = []
        for shape_data in self.shapes:
            color = tuple(float_to_ubyte(f) for f in shape_data.color)
            for p1, p2, p3 in shape_data.local_triangles:
                if shape_data.shading == 'flat':
                    n1 = n2 = n3 = 0.0, 0.0, 1.0
                elif shape_data.shading == 'vertex':
                    n1, n2, n3 = get_vertex_normals(p1, p2, p3)
                elif shape_data.shading == 'face':
                    n1 = n2 = n3 = get_face_normal(p1, p2, p3)
                elif shape_data.shading == 'edge':
                    n1 = 0.0, 0.0, 1.0
                    n2 = n3 = get_face_normal(p1, p2, p3)
                else:
                    assert False
                vertices.extend(p1 + p2 + p3)
                normals.extend(float_to_byte(f) for f in n1 + n2 + n3)
                colors.extend(color * 3)
        if not vertices:
            return None
        return pyglet.graphics.vertex_list(len(vertices) // 2,
                                           ('v2f', vertices),
                                           ('n3b', normals),
                                           ('c3B', colors))

class ShapeData(object):
    def __init__(self, actor, shape, color=(1.0, 1.0, 1.0), shading='vertex'):
        assert isinstance(actor, Actor)
        assert isinstance(shape, b2Shape)
        self.actor = actor
        self.actor.shapes.add(self)
        self.shape = shape
        self.shape.userData = self
        self.color = color
        self.shading = shading

    def delete(self):
        if self.shape is not None:
            self.shape.userData = None
            self.shape.GetBody().DestroyShape(self.shape)
            self.shape = None
        if self.actor is not None:
            self.actor.shapes.remove(self)
            self.actor = None

    @property
    def local_triangles(self):
        if isinstance(self.shape, b2PolygonShape):
            return get_polygon_triangles(self.shape.vertices)
        elif isinstance(self.shape, b2CircleShape):
            return get_circle_triangles(self.shape.localPosition.tuple(),
                                        self.shape.radius)
        else:
            assert False

    @property
    def world_triangles(self):
        if isinstance(self.shape, b2PolygonShape):
            vertices = [self.shape.body.GetWorldPoint(v)
                        for v in self.shape.vertices]
            return get_polygon_triangles(vertices)
        elif isinstance(self.shape, b2CircleShape):
            center = self.shape.body.GetWorldPoint(self.shape.localPosition)
            return get_circle_triangles(center, self.shape.radius)
        else:
            assert False

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
                            restitution=0.0, group_index=0, sensor=False,
                            color=(1.0, 1.0, 1.0), shading='vertex'):
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
        shape_def.isSensor = sensor
        shape = body_data.body.CreateShape(shape_def).asCircle()
        body_data.body.SetMassFromShapes()
        return ShapeData(self, shape, color=color, shading=shading)

    def create_polygon_shape(self, body_data, vertices=None, density=0.0,
                             friction=0.2, restitution=0.0, group_index=0,
                             sensor=False, color=(1.0, 1.0, 1.0), shading='vertex'):
        assert isinstance(body_data, BodyData)
        assert body_data.body is not None
        assert body_data.actor is self
        if vertices is None:
            vertices = get_box_vertices()
        shape_def = b2PolygonDef()
        shape_def.vertices = vertices
        shape_def.density = density
        shape_def.friction = friction
        shape_def.restitution = restitution
        shape_def.filter.groupIndex = group_index
        shape_def.isSensor = sensor
        shape = body_data.body.CreateShape(shape_def).asPolygon()
        body_data.body.SetMassFromShapes()
        return ShapeData(self, shape, color=color, shading=shading)

    def create_revolute_joint(self, body_data_1, body_data_2, anchor,
                              motor_speed=0.0, max_motor_torque=0.0):
        joint_def = b2RevoluteJointDef()
        joint_def.Initialize(body_data_1.body, body_data_2.body, anchor)
        joint_def.motorSpeed = motor_speed
        joint_def.maxMotorTorque = max_motor_torque
        joint = self.game_engine.world.CreateJoint(joint_def).asRevoluteJoint()
        return JointData(self, joint)

    def create_distance_joint(self, body_data_1, body_data_2, anchor_1,
                              anchor_2):
        joint_def = b2DistanceJointDef()
        joint_def.Initialize(body_data_1.body, body_data_2.body, anchor_1,
                             anchor_2)
        joint = self.game_engine.world.CreateJoint(joint_def).asDistanceJoint()
        return JointData(self, joint)

    def draw(self):
        for body_data in self.bodies:
            body_data.draw()

    def on_key_press(self, key, modifiers):
        pass

    def on_key_release(self, key, modifiers):
        pass

class WaterActor(Actor):
    def __init__(self, game_engine, vertices=None, color=(0.0, 0.5, 1.0)):
        super(WaterActor, self).__init__(game_engine)
        body_data = self.create_body()
        shape_data = self.create_polygon_shape(body_data, vertices, sensor=True)
        self.surface_y = body_data.body.position.y
        self.vertices = shape_data.shape.asPolygon().vertices
        self.color = color

    def draw(self):
        with self.game_engine.water_shader as shader:
            shader.uniformf('time', self.game_engine.time)
            shader.uniformf('surface_y', self.surface_y)
            shader.uniformf('wave_height', 0.3)
            shader.uniformf('wave_length', 2.0)
            shader.uniformf('wave_speed', 0.15)
            glColor3f(*self.color)
            glBegin(GL_POLYGON)
            for x, y in self.vertices:
                glTexCoord2f(x, y)
                glVertex2f(x, y)
            glEnd()

class TestPlatformActor(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0), angle=0.0):
        super(TestPlatformActor, self).__init__(game_engine)
        body_data = self.create_body(position=position, angle=angle)
        self.create_polygon_shape(body_data, vertices=get_box_vertices(5.0, 0.1))

class TestVehicleActor(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0)):
        super(TestVehicleActor, self).__init__(game_engine)
        position = b2Vec2(position[0], position[1])
        self.frame_data = self.create_body(position=position)
        self.create_polygon_shape(self.frame_data, vertices=get_box_vertices(1.0, 0.5),
                              density=1000.0, group_index=-self.group_index, color=(1.0, 0.0, 0.0))
        self.left_wheel_data = self.create_body(position=(position + b2Vec2(-0.7, -0.4)))
        self.create_circle_shape(self.left_wheel_data, radius=0.4, density=100.0, friction=5.0,
                                 group_index=-self.group_index, color=(0.5, 0.5, 0.5), shading='edge')
        self.left_joint_data = self.create_revolute_joint(self.frame_data, self.left_wheel_data,
                                                          self.left_wheel_data.body.GetWorldCenter(),
                                                          motor_speed=-20.0, max_motor_torque=7000.0)
        self.right_wheel_data = self.create_body(position=(position + b2Vec2(0.7, -0.4)))
        self.create_circle_shape(self.right_wheel_data, radius=0.4, density=100.0, friction=5.0,
                                 group_index=-self.group_index, color=(0.5, 0.5, 0.5), shading='edge')
        self.right_joint_data = self.create_revolute_joint(self.frame_data, self.right_wheel_data,
                                                      self.right_wheel_data.body.GetWorldCenter(),
                                                      motor_speed=20.0, max_motor_torque=7000.0)

    def on_key_press(self, key, modifiers):
        if key == pyglet.window.key.LEFT:
            self.right_joint_data.joint.enableMotor = True
            self.right_wheel_data.body.WakeUp()
        if key == pyglet.window.key.RIGHT:
            self.left_joint_data.joint.enableMotor = True
            self.left_wheel_data.body.WakeUp()

    def on_key_release(self, key, modifiers):
        if key == pyglet.window.key.LEFT:
            self.right_joint_data.joint.enableMotor = False
        if key == pyglet.window.key.RIGHT:
            self.left_joint_data.joint.enableMotor = False

# TODO: Extract class Camera. The CameraActor should hold a camera instance.
class CameraActor(Actor):
    def __init__(self, game_engine):
        super(CameraActor, self).__init__(game_engine)
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
        self.camera_actor = CameraActor(self)
        self._init_test_actors()
        self.background_image = pyglet.resource.image('resources/images/cave.jpg')
        self.background_image.anchor_x = self.background_image.width // 2
        self.background_image.anchor_y = self.background_image.height // 2
        # self.lighting_image = pyglet.resource.image('cave-lighting.png')

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
        DirectionalLight(self.lighting_manager, color=(1.0, 1.0, 1.0),
                         direction=(1.0, -1.0, -1.0))
        DirectionalLight(self.lighting_manager, color=(0.5, 0.5, 0.5),
                         direction=(-1.0, -1.0, -1.0))
        DirectionalLight(self.lighting_manager, color=(0.1, 0.1, 0.1),
                         direction=(-1.0, 1.0, -1.0))
        DirectionalLight(self.lighting_manager, color=(0.2, 0.2, 0.2),
                         direction=(1.0, 1.0, -1.0))

    def _init_world(self):
        aabb = b2AABB()
        aabb.lowerBound = -1000.0, -1000.0
        aabb.upperBound = 1000.0, 1000.0
        gravity = 0.0, -10.0
        self.world = b2World(aabb, gravity, True)

    def _init_test_actors(self):
        TestPlatformActor(self, angle=-0.2)
        TestPlatformActor(self, position=(9.0, -2.0), angle=0.1)
        WaterActor(self, vertices=get_box_vertices(half_width=10.0))
        self.player_actor = TestVehicleActor(self, position=(-3.0, 3.0))

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
        self.background_image.blit(width // 2, height // 2)
        if self.player_actor is not None:
            self.camera_actor.position = self.player_actor.first_body_position
        with self.camera_actor.transform(width, height):
            glPushAttrib(GL_ALL_ATTRIB_BITS)
            with self.lighting_manager:
                for actor in self.actors:
                    actor.draw()
            glPopAttrib()
            if self.debug_draw is not None:
                self.debug_draw.draw()

    def on_key_press(self, key, modifiers):
        if self.player_actor is not None:
            self.player_actor.on_key_press(key, modifiers)

    def on_key_release(self, key, modifiers):
        if self.player_actor is not None:
            self.player_actor.on_key_release(key, modifiers)

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

class MyShader(shader.Shader):
    def __enter__(self):
        self.bind()
        return self

    def __exit__(self, *args):
        self.unbind()

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
