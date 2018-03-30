import numpy as np
from collections import deque


class MovementHandler:
    """Handles bulk movements for the player (beyond the kickoff).
        Allows player actions to call functions to 'move to point', 'dodge into ball', etc."""

    def __init__(self, player):
        self.player = player
        self.current_movement = None
        self.data = {}
        self.last_steers = deque([0], maxlen=10)

    def reset(self):
        self.current_movement = None
        self.data = {}

    def do_flip(self, flip_pitch=-1, flip_yaw=0, boost=True, held_pitch=0, held_yaw=0):
        if 'flipped' not in self.data:
            self.data['flipped'] = 0
            self.output_vector[5] = 1
        else:
            # print('Flipped:', self.data['flipped'])
            if self.player.car.jumped and self.data['flipped'] == 0:
                # still holding jump
                self.output_vector[5] = 0  # let go of jump
                if time.time() - self.player.car.jumped_time > self.player.car.flip_after_jump_threshold:
                    self.data['flipped'] = 1
            elif self.data['flipped'] == 1:
                # do flip
                self.output_vector[2] = flip_pitch
                self.output_vector[3] = flip_yaw
                self.output_vector[5] = 1

                self.data['flipped'] = 2
            elif self.data['flipped'] == 2:
                self.output_vector[2] = held_pitch
                self.output_vector[3] = held_yaw
                self.output_vector[5] = 0

                if self.player.car.on_ground:
                    self.increment_stage()

        if boost:
            # check if car is not pointing up
            pitch_limit = 300
            if -pitch_limit < self.player.car.rotation[0] < pitch_limit:
                self.set_boost_if_not_near_max()

    def set_boost_if_not_near_max(self):
        # boost only if not moving at max speed
        if np.isclose(self.player.car.speed, self.player.car.car_max_speed):
            self.output_vector[6] = 0
        else:
            self.output_vector[6] = 1

    def half_flip(self):
        pass

    def move_to_point(self, point, boost=False):
        """Finds next stage to move to point."""
        output_vector = [0, 0, 0, 0, 0, 0, 0, 0]  # throttle steer pitch yaw roll jump boost handbrake
        # if point is > 400 behind and player forward speed < limit, half flip

        # if point within turn radius (cannot be reached by turning), use handbrake turn,
        _time = self.player.car.find_time_to_face_point(point)
        if _time != 0:
            time_to_face_point, steer, remaining_distance = _time
            if np.isnan(remaining_distance):
                output_vector[7] = 1  # use handbrake turn
                output_vector[0] = -0.3  # brake/throttle back
                output_vector[1] = steer

            # else turn to ball, move forward
            else:
                variance_of_last_steer = np.var(self.last_steers)

                steer_multiple = np.exp(-remaining_distance / 500 * variance_of_last_steer)
                if not np.isnan(steer_multiple):
                    steer = steer * steer_multiple

                output_vector[0] = 1
                output_vector[6] = boost
                output_vector[1] = steer
