import numpy as np
import scipy.integrate as spi
import matplotlib.pyplot as plt


class Ball:
    def __init__(self):
        self.ball_radius = 92.775

        self.air_resistance = 0.0305  # % loss per second
        self.rolling_resistance = 0.022  # loss per second of rotation

        self.sliding_friction = 230  # uu/s2
        # if ball is rolling less than its perimeter per second, no sliding friction
        # this is a speed limit under which there is no sliding friction
        self.sliding_friction_limit = 565  # uu/s

        # ball stop conditions
        self.ball_stop_speed = 40  # uu/s
        self.ball_stop_spin = 10  # rpm

        self.gravity = 650  # uu/s2

        self.ball_max_speed = 6000

        self.restitution = 0.6

        self.position = np.array([0, 0, 0], dtype=np.float32)
        self.velocity = np.array([0, 0, 0], dtype=np.float32)
        self.speed = 0
        self.angular_velocity = np.array([0, 0, 0], dtype=np.float32)

    def predict_ball_positions(self, t=10, points=None):
        """Returns an array of time and position and velocity up to time=t."""
        if points is None:
            points = t * 10
        _t = np.linspace(0, t, num=points)
        starting_x_v = np.concatenate((self.position, self.velocity))
        x_v = spi.odeint(self.find_dt, starting_x_v, _t)
        # TODO: COMPLETE THIS FUNCTION.
        plt.plot(_t, x_v[:, 2])
        plt.show()
        return _t, x_v

    def find_dt(self, x_v, t):
        x = x_v[:3]
        v = x_v[3:]

        # calculate collisions
        collided = False
        # floor
        if x[2] < self.ball_radius or x[2] > self.player.stadium.ceiling_distance - self.ball_radius:
            if self.check_if_ball_leaving(x_v):
                collided = True
                normal_vector = np.array([0, 0, 1])
        # sides
        if x[0] < -self.player.stadium.side_wall_distance + self.ball_radius or \
                x[0] > self.player.stadium.side_wall_distance - self.ball_radius:
            if self.check_if_ball_leaving(x_v):
                collided = True
                normal_vector = np.array([1, 0, 0])
        # back
        if x[1] < -self.player.stadium.back_wall_distance + self.ball_radius or \
                x[1] > self.player.stadium.back_wall_distance - self.ball_radius:
            if self.check_if_ball_leaving(x_v):
                collided = True
                normal_vector = np.array([0, 1, 0])
        if collided:
            v_perp = v.dot(normal_vector)
            # TODO: CORRECT NORMAL VECTOR DIRECTION
            v_parallel = v - v_perp
            v_perp = -v_perp * self.restitution
            v_parallel = v_parallel * 0.75
            v = v_perp + v_parallel

        # calculate a
        a = np.array([0, 0, -self.gravity]) - self.air_resistance * v

        # if v > max speed: v = v
        if v.dot(v) > self.ball_max_speed ** 2:
            v = v / np.sqrt(v.dot(v)) * self.ball_max_speed

        return np.concatenate((v, a))

    def check_if_ball_leaving(self, x_v):
        # if x (-[0, 1000, 0] to move origin to centre of stadium)  and v in same direction: leaving
        ball_displacement_from_centre = x_v[0:3] - np.array([0, 0, 1000])
        if ball_displacement_from_centre.dot(x_v[3:6]) > 0:
            return True
        else:
            return False

    def update(self, game_tick_packet):
        self.position[:] = (game_tick_packet.gameball.Location.X,
                            game_tick_packet.gameball.Location.Y,
                            game_tick_packet.gameball.Location.Z)
        self.velocity[:] = (game_tick_packet.gameball.Velocity.X,
                            game_tick_packet.gameball.Velocity.Y,
                            game_tick_packet.gameball.Velocity.Z)
        self.speed = np.sqrt(self.velocity.dot(self.velocity))
        self.angular_velocity[:] = (game_tick_packet.gameball.AngularVelocity.X,
                                    game_tick_packet.gameball.AngularVelocity.Y,
                                    game_tick_packet.gameball.AngularVelocity.Z)
