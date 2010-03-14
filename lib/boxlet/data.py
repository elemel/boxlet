import boxlet.actors
from boxlet.utils import *

from Box2D import *
from pyglet.gl import *

class BodyData(object):
    def __init__(self, actor, body):
        assert isinstance(actor, boxlet.actors.Actor)
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
                normals.extend(map(float_to_byte, n1 + n2 + n3))
                colors.extend(color * 3)
        if not vertices:
            return None
        return pyglet.graphics.vertex_list(len(vertices) // 2,
                                           ('v2f', vertices),
                                           ('n3b', normals),
                                           ('c3B', colors))

class ShapeData(object):
    def __init__(self, actor, shape, color=(1.0, 1.0, 1.0), shading='vertex'):
        assert isinstance(actor, boxlet.actors.Actor)
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
        assert isinstance(actor, boxlet.actors.Actor)
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
