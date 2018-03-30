import numpy as np
import time
from collections import deque
from quicktracer import trace


def unrotate_positions(relative_positions, rotations):
    new_positions = relative_positions

    # YAW
    yaws = rotations[:, 1]
    yaws = -yaws / 32768. * np.pi

    new_positions[:, 0], new_positions[:, 1] = new_positions[:, 0] * np.cos(yaws) - new_positions[:, 1] * np.sin(
        yaws), new_positions[:, 0] * np.sin(yaws) + new_positions[:, 1] * np.cos(yaws)

    # PITCH

    pitchs = rotations[:, 0]
    pitchs = pitchs / 32768. * np.pi

    new_positions[:, 2], new_positions[:, 0] = new_positions[:, 2] * np.cos(pitchs) - new_positions[:, 0] * np.sin(
        pitchs), new_positions[:, 2] * np.sin(pitchs) + new_positions[:, 0] * np.cos(pitchs)

    # ROLL

    rolls = rotations[:, 2]
    rolls = rolls / 32768. * np.pi

    new_positions[:, 1], new_positions[:, 2] = new_positions[:, 1] * np.cos(rolls) - new_positions[:, 2] * np.sin(
        rolls), new_positions[:, 1] * np.sin(rolls) + new_positions[:, 2] * np.cos(rolls)

    return new_positions


class Car:

    def __init__(self, index):
        """Class takes care of all car simulation. Pass agent's index to init to parse game_tick_packets."""
        self.index = index
        self.supersonic_speed = 2200
        self.car_max_speed = 2300  # uu/s
        self.car_max_speed_without_boost = 1400

        self.length = 0
        self.width = 0
        self.height = 0
        self.position = np.array([0, 0, 0], dtype=np.float32)
        self.rotation = np.array([0, 0, 0], dtype=np.float32)
        self.velocity = np.array([0, 0, 0], dtype=np.float32)
        self.speed = 0
        self.angular_velocity = np.array([0, 0, 0], dtype=np.float32)
        
        self.score = None
        self.demolished = None
        self.on_ground = None
        self.supersonic = None
        self.jumped = None
        self.doublejumped = None
        self.name = None
        self.team = None
        self.boost = None

        self.jumped_time = None  # None when not jumped, set to time when just jumped
        self.flip_after_jump_threshold = 0.06
        
        self.last_steers = deque([0], maxlen=10)

    def update(self, game_tick_packet):
        car_player_info = game_tick_packet.gamecars[self.index]
        self.position[:] = (car_player_info.Location.X,
                            car_player_info.Location.Y,
                            car_player_info.Location.Z)
        self.rotation[:] = (car_player_info.Rotation.Pitch,
                            car_player_info.Rotation.Yaw,
                            car_player_info.Rotation.Roll)
        self.velocity[:] = (car_player_info.Velocity.X,
                            car_player_info.Velocity.Y,
                            car_player_info.Velocity.Z)
        self.speed = np.sqrt(self.velocity.dot(self.velocity))
        self.angular_velocity[:] = (car_player_info.AngularVelocity.X,
                                    car_player_info.AngularVelocity.Y,
                                    car_player_info.AngularVelocity.Z)
        
        self.score = car_player_info.Score
        self.demolished = car_player_info.bDemolished
        self.on_ground = car_player_info.bOnGround
        self.supersonic = car_player_info.bSuperSonic
        self.jumped = car_player_info.bJumped
        self.doublejumped = car_player_info.bDoubleJumped
        self.name = car_player_info.wName
        self.team = car_player_info.Team
        self.boost = car_player_info.Boost

        # set jumped_time to time if just jumped
        if self.jumped_time is None and self.jumped:
            self.jumped_time = time.time()
        if not self.jumped:
            self.jumped_time = None

    def get_turn_radius(self, speed=None):
        if speed is None:
            radius = .0153 * self.speed ** 2 + .16 * self.speed + 7
        else:
            radius = .0153 * speed ** 2 + .16 * speed + 7
        return radius

    def get_velocity_at_time(self):
        # v = 1600 * (1 - e^(throttle * t))
        pass

    def get_distance_at_time(self):
        # s = 1600 * (t + (e^(-throttle * t) / throttle))
        # s = 1600 / throttle * (throttle * t + 1 + e^(-throttle * t))
        pass

    def find_time_to_point(self, point, use_boost=False):
        _time = self.find_time_to_face_point(point)
        if _time != 0:
            time_to_face_point, steer, remaining_distance = _time
        else:
            time_to_face_point = 0

        if np.isnan(remaining_distance):
            # Cannot reach ball by turning at current speed

            # Approximate remaining distance as current distance, turning time as 2s
            remaining_distance = self.get_distance_to_ball()
            time_to_face_point = 2.

            max_speed = self.car_max_speed if use_boost else self.car_max_speed_without_boost
            total_time = time_to_face_point + remaining_distance / max_speed
            return total_time, steer
        else:
            max_speed = self.car_max_speed if use_boost else self.car_max_speed_without_boost
            total_time = time_to_face_point + remaining_distance / max_speed

            # reduce steer if remaining_distance is high and variance of last steer is high
            variance_of_last_steer = np.var(self.last_steers)
            # trace(variance_of_last_steer)

            steer_multiple = np.exp(-remaining_distance / 500 * variance_of_last_steer)
            steer = steer * steer_multiple

            # trace(steer_multiple)
            # print(steer_multiple, variance_of_last_steer, remaining_distance)

            return total_time, steer

    def find_time_to_face_point(self, point):
        """For given point and speed, find time car will take to face point.
        Returns time, required steer, and remaining distance after turn"""
        steer = None
        # solve equation for minimum time to face point
        # find equation for turn circle (x)^2 + (y - y0)^2 = r^2
        r = self.get_turn_radius(self.speed)
        relative_point = point - self.position
        unrotated_point = unrotate_positions(relative_point.reshape(1, -1), self.rotation.reshape(1, -1))[0]
        yaw_difference = np.arctan2(unrotated_point[1], -unrotated_point[
            0])  # this is normally atan2(y, x) but left-handed axis with x vertical

        if np.absolute(yaw_difference) < 0.5:
            steer = 0
            remaining_distance = np.sqrt(np.sum(unrotated_point[:2]**2))
            t = 0
        else:
            # find point on circle where tangent meets target point
            t_x = unrotated_point[0]
            t_y = unrotated_point[1]

            # TODO: Handle when ball not in circle
            if t_y < 0:
                # ball on left - take right point
                steer = -1
                r = -r
                p_y = (r * t_y ** 2 - r ** 2 * t_y + t_x ** 2 * r + np.sqrt(
                    r ** 2 * t_x ** 4 - 2 * t_y * r ** 3 * t_x ** 2 + t_y ** 2 * r ** 2 * t_x ** 2)) / (
                              t_y ** 2 - 2 * r * t_y + t_x ** 2 + r ** 2)
                p_x = np.sqrt(r ** 2 - (p_y - r) ** 2)
                r = -r  # reset r
                # find time to that point on circle
                theta = np.arctan2(p_x, p_y - r)

            if t_y > 0:
                # ball on right - take left point
                steer = 1
                p_y = (r * t_y ** 2 - r ** 2 * t_y + t_x ** 2 * r - np.sqrt(
                    r ** 2 * t_x ** 4 - 2 * t_y * r ** 3 * t_x ** 2 + t_y ** 2 * r ** 2 * t_x ** 2)) / (
                              t_y ** 2 - 2 * r * t_y + t_x ** 2 + r ** 2)
                p_x = np.sqrt(r ** 2 - (p_y - r) ** 2)
                theta = np.arctan2(p_x, p_y - r)

            remaining_distance = np.sqrt((p_x - t_x) ** 2 + (p_y - t_y) ** 2)

            # TODO: Handle when speed==0
            t = theta * r / self.speed if self.speed != 0 else theta * r

        return t, steer, remaining_distance

    def get_distance_to_ball(self):
        displacement = self.player.ball.position - self.position
        return np.sqrt(displacement.dot(displacement))


class Octane(Car):

    def __init__(self, index):
        super.__init__(index)
        self.length = 118.01
        self.width = 84.20
        self.height = 36.16

        self.rj_x_offset = -13.88
        self.rj_z_offset = -20.75
