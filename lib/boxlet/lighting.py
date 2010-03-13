from pyglet.gl import *

class LightingManager(object):
    def __init__(self, count=8, enabled=False):
        self._alloc = iter(xrange(GL_LIGHT0, GL_LIGHT0 + count)).next
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
            return self._alloc()

    def free(self, light_name):
        self._free.append(light_name)

class LightImpl(object):
    def __init__(self, lighting_manager, ambient=(0.0, 0.0, 0.0, 1.0),
                 diffuse=(0.0, 0.0, 0.0, 1.0), specular=(0.0, 0.0, 0.0, 1.0),
                 position=(0.0, 0.0, 1.0, 0.0), enabled=True):
        assert isinstance(lighting_manager, LightingManager)
        self._lighting_manager = lighting_manager
        self._light_name = self._lighting_manager.alloc()
        self._enabled = False
        self.ambient = ambient
        self.diffuse = diffuse
        self.specular = specular
        self.position = position
        self.enabled = enabled

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

    @property
    def ambient(self):
        raise NotImplementedError()

    @ambient.setter
    def ambient(self, ambient):
        glLightfv(self._light_name, GL_AMBIENT, (c_float * 4)(*ambient))

    @property
    def diffuse(self):
        raise NotImplementedError()

    @diffuse.setter
    def diffuse(self, diffuse):
        glLightfv(self._light_name, GL_DIFFUSE, (c_float * 4)(*diffuse))

    @property
    def specular(self):
        raise NotImplementedError()

    @specular.setter
    def specular(self, specular):
        glLightfv(self._light_name, GL_SPECULAR, (c_float * 4)(*specular))

    @property
    def position(self):
        raise NotImplementedError()

    @position.setter
    def position(self, position):
        glLightfv(self._light_name, GL_POSITION, (c_float * 4)(*position))

class Light(object):
    def __init__(self, impl):
        assert isinstance(impl, LightImpl)
        self._impl = impl

    def delete(self):
        if self._impl is not None:
            self._impl.delete()
            self._impl = None

    @property
    def enabled(self):
        return self._impl.enabled

    @enabled.setter
    def enabled(self, enabled):
        self._impl.enabled = enabled

class DirectionalLight(Light):
    def __init__(self, lighting_manager, color=(1.0, 1.0, 1.0),
                 direction=(0.0, 0.0, -1.0), enabled=True):
        assert isinstance(lighting_manager, LightingManager)
        diffuse = specular = color + (1.0,)
        position = (-direction[0], -direction[1], -direction[2])
        impl = LightImpl(lighting_manager, diffuse=diffuse,
                         specular=specular, position=position, enabled=enabled)
        super(DirectionalLight, self).__init__(impl)
