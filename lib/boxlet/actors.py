import boxlet.controllers
import boxlet.game_engine
import boxlet.physics
from boxlet.utils import *

from Box2D import *
from pyglet.gl import *
import random

class Actor(object):
    def __init__(self, game_engine):
        assert isinstance(game_engine, boxlet.game_engine.GameEngine)
        self.game_engine = game_engine
        self.game_engine.actors.add(self)
        self.group_index = game_engine.generate_group_index()
        self.bodies = set()
        self.shapes = set()
        self.joints = set()
        self.controllers = set()

    def delete(self):
        for controller in list(self.controllers):
            controller.delete()
        assert not self.controllers
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

    def draw(self):
        for body_data in self.bodies:
            body_data.draw()

    def on_key_press(self, key, modifiers):
        pass

    def on_key_release(self, key, modifiers):
        pass

class WaterActor(Actor):
    def __init__(self, game_engine, vertices=None, color=(0.0, 1.0, 2.0),
                 opacity=0.5):
        super(WaterActor, self).__init__(game_engine)
        body_data = boxlet.physics.BodyData(actor=self)
        shape_data = boxlet.physics.PolygonShapeData(actor=self,
                                                     body_data=body_data,
                                                     vertices=vertices,
                                                     sensor=True)
        self.surface_y = body_data.body.position.y
        self.vertices = shape_data.shape.asPolygon().vertices
        self.color = color
        self.opacity = opacity

    def draw(self):
        with self.game_engine.water_shader as shader:
            shader.uniformf('time', self.game_engine.time)
            shader.uniformf('surface_y', self.surface_y)
            shader.uniformf('wave_height', 0.3)
            shader.uniformf('wave_length', 2.0)
            shader.uniformf('wave_speed', 0.5)
            r, g, b = self.color
            glColor4f(r, g, b, self.opacity)
            glBegin(GL_POLYGON)
            for x, y in self.vertices:
                glTexCoord2f(x, y)
                glVertex2f(x, y)
            glEnd()

class ParticleActor(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0)):
        super(ParticleActor, self).__init__(game_engine)
        self.position = position

class FireActor(ParticleActor):
    def __init__(self, game_engine, position):
        super(FireActor, self).__init__(game_engine, position)
        boxlet.controllers.ParticleGravityController(self, (0.0, 15.0))
        self.emit()

    def emit(self):
        linear_velocity = random.uniform(-2.0, 2.0), random.uniform(-2.0, 2.0)
        body_data = boxlet.physics.BodyData(actor=self, position=self.position,
                                            linear_velocity=linear_velocity,
                                            linear_damping=2.0)
        boxlet.physics.CircleShapeData(actor=self, body_data=body_data,
                                       radius=0.05, density=100.0, sensor=True)
        body_data.time = self.game_engine.time
        self.game_engine.schedule(1.0, body_data.delete)
        self.game_engine.schedule(1.0 / 60.0, self.emit)

    def draw(self):
        glPushAttrib(GL_COLOR_BUFFER_BIT)
        glBlendFunc(GL_ONE, GL_ONE)
        glPointSize(20.0)
        glBegin(GL_POINTS)
        for body_data in self.bodies:
            time = self.game_engine.time - body_data.time
            glColor3f(1.0 - time, 0.5 - time, 0.2 - time)
            glVertex2f(body_data.body.position.x, body_data.body.position.y)
        glEnd()
        glPopAttrib()

class TestPlatformActor(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0), angle=0.0):
        super(TestPlatformActor, self).__init__(game_engine)
        body_data = boxlet.physics.BodyData(actor=self, position=position,
                                            angle=angle)
        boxlet.physics.PolygonShapeData(actor=self, body_data=body_data,
                                        vertices=get_box_vertices(5.0, 0.1),
                                        color=(1.0, 0.5, 0.0))

class TestVehicleActor(Actor):
    def __init__(self, game_engine, position=(0.0, 0.0)):
        super(TestVehicleActor, self).__init__(game_engine)
        position = b2Vec2(position[0], position[1])
        self.frame_data = boxlet.physics.BodyData(actor=self,
                                                  position=position)
        boxlet.physics.PolygonShapeData(actor=self, body_data=self.frame_data,
                                        vertices=[(-1.0, -0.5), (1.0, -0.5),
                                                  (0.5, 0.7), (-0.5, 0.7)],
                                        density=1000.0,
                                        group_index=-self.group_index,
                                        color=(1.0, 1.0, 1.0))
        self.left_wheel_data = boxlet.physics.BodyData(actor=self,
                                                       position=(position +
                                                                 b2Vec2(-0.7,
                                                                        -0.4)))
        boxlet.physics.CircleShapeData(actor=self, body_data=self.left_wheel_data,
                                       radius=0.4, density=100.0, friction=5.0,
                                       group_index=-self.group_index,
                                       color=(0.6, 0.8, 0.4), shading='edge')
        self.left_joint_data = boxlet.physics.RevoluteJointData(actor=self, body_data_1=self.frame_data, body_data_2=self.left_wheel_data,
                                                                anchor=self.left_wheel_data.body.GetWorldCenter(),
                                                                motor_speed=-20.0, max_motor_torque=7000.0)
        self.right_wheel_data = boxlet.physics.BodyData(actor=self, position=(position + b2Vec2(0.7, -0.4)))
        boxlet.physics.CircleShapeData(actor=self,
                                    body_data=self.right_wheel_data,
                                    radius=0.4, density=100.0, friction=5.0,
                                    group_index=-self.group_index,
                                    color=(0.6, 0.8, 0.4), shading='edge')
        self.right_joint_data = boxlet.physics.RevoluteJointData(actor=self, body_data_1=self.frame_data, body_data_2=self.right_wheel_data,
                                                                 anchor=self.right_wheel_data.body.GetWorldCenter(),
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
