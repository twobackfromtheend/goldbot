from player.player import Player
from model.car import Car
from model.stadium import Stadium
from model.ball import Ball


class Agent:
    def __init__(self, name, team, index):
        self.name = name
        self.team = team  # 0 towards positive goal, 1 towards negative goal.
        self.index = index
        self.car = Car(index)
        self.stadium = Stadium()
        self.ball = Ball()
        self.player = Player(self.car, self.stadium, self.ball)
        self.last_game_tick_packet = None

    def get_output_vector(self, game_tick_packet):
        if game_tick_packet == self.last_game_tick_packet:
            return [0, 0, 0, 0, 0, 0, 0, 0]

        self.player.update(game_tick_packet)
        return self.player.get_output_vector()


        """
        inputs = [game_tick_packet.gamecars[self.index].Location.X,
                  game_tick_packet.gamecars[self.index].Location.Y,
                  game_tick_packet.gamecars[self.index].Location.Z,
                  game_tick_packet.gamecars[self.index].Rotation.Pitch,
                  game_tick_packet.gamecars[self.index].Rotation.Yaw,
                  game_tick_packet.gamecars[self.index].Rotation.Roll,
                  game_tick_packet.gamecars[self.index].Velocity.X,
                  game_tick_packet.gamecars[self.index].Velocity.Y,
                  game_tick_packet.gamecars[self.index].Velocity.Z,
                  game_tick_packet.gamecars[self.index].AngularVelocity.X,
                  game_tick_packet.gamecars[self.index].AngularVelocity.Y,
                  game_tick_packet.gamecars[self.index].AngularVelocity.Z,
                  game_tick_packet.gamecars[self.index].Boost,
                  game_tick_packet.gameball.Location.X,
                  game_tick_packet.gameball.Location.Y,
                  game_tick_packet.gameball.Location.Z,
                  game_tick_packet.gameball.Velocity.X,
                  game_tick_packet.gameball.Velocity.Y,
                  game_tick_packet.gameball.Velocity.Z
                  ]
        inputs = np.array(inputs).reshape((1, -1))

        relative_positions = inputs[:, 13:16] - inputs[:, 0:3]
        rotations = inputs[:, 3:6]
        unrotated_positions = utils.unrotate_positions(relative_positions, rotations)

        inputs = np.column_stack((inputs, unrotated_positions))


        if unrotated_positions[1] < 0:
            steer = -1
        else:
            steer = 1
        return [1.0, steer, 0.0, 0.0, 0.0, 0, 0, 0]
        """
