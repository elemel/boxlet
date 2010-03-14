from Box2D import *
from contextlib import contextmanager
import math
from pyglet.gl import *

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

    @contextmanager
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
