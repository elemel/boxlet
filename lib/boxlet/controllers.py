import boxlet.actors
import boxlet.game_engine

from Box2D import *

class Controller(object):
    def __init__(self, game_engine):
        assert isinstance(game_engine, boxlet.game_engine.GameEngine)
        self.game_engine = game_engine
        self.game_engine.controllers.add(self)

    def delete(self):
        if self.game_engine is not None:
            self.game_engine.controllers.remove(self)
            self.game_engine = None

    def step(self, dt):
        pass

class ActorController(Controller):
    def __init__(self, actor):
        assert isinstance(actor, boxlet.actors.Actor)
        super(ActorController, self).__init__(actor.game_engine)
        self.actor = actor

    def delete(self):
        if self.actor is not None:
            self.actor.controllers.remove(self)
            self.actor = None
        super(ActorController, self).delete()

class ParticleGravityController(ActorController):
    def __init__(self, actor, gravity):
        super(ParticleGravityController, self).__init__(actor)
        self.gravity = b2Vec2(*gravity)

    def step(self, dt):
        for body_data in self.actor.bodies:
            body_data.body.linearVelocity += dt * self.gravity
