import boxlet.game_engine

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
