from pyglet.gl import *

CFloat3 = c_float * 3
CFloat4 = c_float * 4

class LightingManager(object):
    def __init__(self, count=8, enabled=False):
        self._alloc = iter(xrange(GL_LIGHT0, GL_LIGHT0 + count))
        self._free = []
        self._enabled = False
        self.enabled = enabled

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, enabled):
        assert isinstance(enabled, bool)
        self._enabled = enabled
        if self._enabled:
            glEnable(GL_LIGHTING)
        else:
            glDisable(GL_LIGHTING)

    def __enter__(self):
        self.enabled = True

    def __exit__(self, *args):
        self.enabled = False

    def alloc(self):
        if self._free:
            return self._free.pop()
        else:
            return self._alloc.next()

    def free(self, light_name):
        self._free.append(light_name)

class Light(object):
    def __init__(self, lighting_manager, enabled=True):
        assert isinstance(lighting_manager, LightingManager)
        self._lighting_manager = lighting_manager
        self._light_name = self._lighting_manager.alloc()
        self.enabled = enabled

        # Reset OpenGL light source parameters. No special handling of
        # GL_LIGHT0 for the diffuse and specular term.
        glLightfv(self._light_name, GL_AMBIENT, CFloat4(0.0, 0.0, 0.0, 1.0))
        glLightfv(self._light_name, GL_DIFFUSE, CFloat4(0.0, 0.0, 0.0, 1.0))
        glLightfv(self._light_name, GL_SPECULAR, CFloat4(0.0, 0.0, 0.0, 1.0))
        glLightfv(self._light_name, GL_POSITION, CFloat4(0.0, 0.0, 1.0, 0.0))
        glLightfv(self._light_name, GL_SPOT_DIRECTION, CFloat3(0.0, 0.0, -1.0))
        glLightf(self._light_name, GL_SPOT_EXPONENT, 0.0)
        glLightf(self._light_name, GL_SPOT_CUTOFF, 180.0)
        glLightf(self._light_name, GL_CONSTANT_ATTENUATION, 1.0)
        glLightf(self._light_name, GL_LINEAR_ATTENUATION, 1.0)
        glLightf(self._light_name, GL_QUADRATIC_ATTENUATION, 1.0)

    def delete(self):
        if self._lighting_manager is not None:
            self.enabled = False
            self._lighting_manager.free(self._light_name)
            self._lighting_manager = None

    @property
    def enabled(self):
        return self._enabled

    @enabled.setter
    def enabled(self, enabled):
        assert isinstance(enabled, bool)
        self._enabled = enabled
        if self._enabled:
            glEnable(self._light_name)
        else:
            glDisable(self._light_name)

class DirectionalLight(Light):
    def __init__(self, lighting_manager, color=(1.0, 1.0, 1.0),
                 direction=(0.0, 0.0, -1.0), enabled=True):
        assert isinstance(lighting_manager, LightingManager)
        super(DirectionalLight, self).__init__(lighting_manager, enabled)
        self.color = color
        self.direction = direction
 
    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, color):
        self._color = color
        r, g, b = self._color
        color = CFloat4(r, g, b, 1.0)
        glLightfv(self._light_name, GL_DIFFUSE, color)
        glLightfv(self._light_name, GL_SPECULAR, color)

    @property
    def direction(self):
        return self._direction

    @direction.setter
    def direction(self, direction):
        self._direction = direction
        x, y, z = self._direction
        position = CFloat4(-x, -y, -z, 0.0)
        glLightfv(self._light_name, GL_POSITION, position)
