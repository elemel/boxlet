import boxlet.data
import boxlet.game_engine
from boxlet.utils import *

from Box2D import *
from pyglet.gl import *

class Actor(object):
    def __init__(self, game_engine):
        assert isinstance(game_engine, boxlet.game_engine.GameEngine)
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
        return boxlet.data.BodyData(self, body)

    def create_circle_shape(self, body_data, local_position=(0.0, 0.0),
                            radius=1.0, density=0.0, friction=0.2,
                            restitution=0.0, group_index=0, sensor=False,
                            color=(1.0, 1.0, 1.0), shading='vertex'):
        assert isinstance(body_data, boxlet.data.BodyData)
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
        return boxlet.data.ShapeData(self, shape, color=color, shading=shading)

    def create_polygon_shape(self, body_data, vertices=None, density=0.0,
                             friction=0.2, restitution=0.0, group_index=0,
                             sensor=False, color=(1.0, 1.0, 1.0), shading='vertex'):
        assert isinstance(body_data, boxlet.data.BodyData)
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
        return boxlet.data.ShapeData(self, shape, color=color, shading=shading)

    def create_revolute_joint(self, body_data_1, body_data_2, anchor,
                              motor_speed=0.0, max_motor_torque=0.0):
        assert isinstance(body_data_1, boxlet.data.BodyData)
        assert isinstance(body_data_2, boxlet.data.BodyData)
        joint_def = b2RevoluteJointDef()
        joint_def.Initialize(body_data_1.body, body_data_2.body, anchor)
        joint_def.motorSpeed = motor_speed
        joint_def.maxMotorTorque = max_motor_torque
        joint = self.game_engine.world.CreateJoint(joint_def).asRevoluteJoint()
        return boxlet.data.JointData(self, joint)

    def create_distance_joint(self, body_data_1, body_data_2, anchor_1,
                              anchor_2):
        assert isinstance(body_data_1, boxlet.data.BodyData)
        assert isinstance(body_data_2, boxlet.data.BodyData)
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
            shader.uniformf('wave_speed', 0.5)
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
        self.create_polygon_shape(body_data, vertices=get_box_vertices(5.0, 0.1), color=(1.0, 0.5, 0.0))

class TestVehicleActor(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0)):
        super(TestVehicleActor, self).__init__(game_engine)
        position = b2Vec2(position[0], position[1])
        self.frame_data = self.create_body(position=position)
        self.create_polygon_shape(self.frame_data, vertices=get_box_vertices(1.0, 0.5),
                              density=1000.0, group_index=-self.group_index, color=(1.0, 1.0, 1.0))
        self.left_wheel_data = self.create_body(position=(position + b2Vec2(-0.7, -0.4)))
        self.create_circle_shape(self.left_wheel_data, radius=0.4, density=100.0, friction=5.0,
                                 group_index=-self.group_index, color=(0.6, 0.8, 0.4), shading='edge')
        self.left_joint_data = self.create_revolute_joint(self.frame_data, self.left_wheel_data,
                                                          self.left_wheel_data.body.GetWorldCenter(),
                                                          motor_speed=-20.0, max_motor_torque=7000.0)
        self.right_wheel_data = self.create_body(position=(position + b2Vec2(0.7, -0.4)))
        self.create_circle_shape(self.right_wheel_data, radius=0.4, density=100.0, friction=5.0,
                                 group_index=-self.group_index, color=(0.6, 0.8, 0.4), shading='edge')
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
