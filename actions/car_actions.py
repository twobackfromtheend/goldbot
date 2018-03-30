import numpy as np
import time


class BaseAction:
    """Actions are calculated to find urgency, time, and output_vector required."""

    def __init__(self, player):
        self.player = player
        self.time_to_execute = None
        self.output_vector = [0, 0, 0, 0, 0, 0, 0, 0, 0]  # throttle steer pitch yaw roll jump boost handbrake
        self.urgency = 0
        self.lock = 0  # 0 when action will be overwritten, 1 when action will not be overwritten

        # used to keep track of continuation of action
        self.stages = []
        self.stage_data = {}
        self.current_stage_index = 0

    def __repr__(self):
        return self.__class__.__name__

    def update_output_vector(self):
        """Called on every game_tick_packet to retrieve the output vector needed to execute this action."""
        pass

    def increment_stage(self):
        """Handles stage index incrementing, empties stage_data, and calls on_complete if stage_index > len(stages)"""
        print('Finished Stage: %s' % self.stages[self.current_stage_index])
        self.current_stage_index += 1
        self.stage_data = {}
        if self.current_stage_index >= len(self.stages):
            self.on_complete()

    def get_stages(self):
        pass

    def on_complete(self):
        self.lock = 0

    def update_urgency(self):
        """To quickly update urgency when no lock is present (ie to choose action to perform)"""
        pass

    def do_flip(self, flip_pitch=-1, flip_yaw=0, boost=True, held_pitch=0, held_yaw=0):
        if 'flipped' not in self.stage_data:
            self.stage_data['flipped'] = 0
            self.output_vector[5] = 1
        else:
            # print('Flipped:', self.stage_data['flipped'])
            if self.player.car.jumped and self.stage_data['flipped'] == 0:
                # still holding jump
                self.output_vector[5] = 0  # let go of jump
                if time.time() - self.player.car.jumped_time > self.player.car.flip_after_jump_threshold:
                    self.stage_data['flipped'] = 1
            elif self.stage_data['flipped'] == 1:
                # do flip
                self.output_vector[2] = flip_pitch
                self.output_vector[3] = flip_yaw
                self.output_vector[5] = 1

                self.stage_data['flipped'] = 2
            elif self.stage_data['flipped'] == 2:
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


class DefenseAction(BaseAction):
    def __init__(self, player):
        super().__init__(player)
        self.urgency = 50


class AttackAction(BaseAction):
    def __init__(self, player):
        super().__init__(player)
        self.urgency = 20


class UtilityAction(BaseAction):
    def __init__(self, player):
        super().__init__(player)
        self.urgency = 50


class MandatoryActions(BaseAction):
    def __init__(self, player):
        super().__init__(player)
        self.lock = 1
        self.urgency = 90


class KickOff(MandatoryActions):
    kickoff_positions = [np.array([-1952, -2464]),
                         np.array([-256, -3840]),
                         np.array([0, -4607]),
                         np.array([256, -3840]),
                         np.array([1952, -2464])]

    # pos_x, pos_y for blue kickoffs - diagonal right, centre right, centre, centre left, diagonal left

    def __init__(self, player):
        super().__init__(player)
        self.output_vector[0] = 1  # mandatory throttle+
        self.output_vector[6] = 1  # mandatory boost
        self.kickoff_label = None  # for centre, 1 for off-centre, 2 for diagonal
        self.stage_data = {}

    def update_output_vector(self):
        if self.kickoff_label is None:
            self.kickoff_label = self.get_kickoff_label()
            self.stages = self.get_stages()

        if self.current_stage_index is not None:
            current_stage = self.stages[self.current_stage_index]

        if self.kickoff_label == 2:
            # centre
            if self.current_stage_index == 0:
                # get centre boost
                # print(self.player.car.boost, self.stage_data.get('last_boost', 100))
                if self.stage_data.get('spawned', False) and self.player.car.boost > self.stage_data.get('last_boost',
                                                                                                         100):
                    self.increment_stage()
                if self.player.car.boost > 10:
                    self.stage_data['spawned'] = True
                self.stage_data['last_boost'] = self.player.car.boost

            elif self.current_stage_index == 1:
                # turn right as long as yaw does not reach limit
                yaw_limit = 16384 * 10 / 9
                # print('car yaw, yaw_limit:', self.player.car.rotation[1], yaw_limit)
                if self.player.car.rotation[1] < yaw_limit:
                    self.output_vector[1] = 1
                    self.output_vector[7] = 1
                else:
                    self.output_vector[1] = 0
                    self.output_vector[7] = 0
                    self.increment_stage()

            elif self.current_stage_index == 2:
                # flip diagonally left
                self.do_flip(flip_pitch=-0.3, flip_yaw=-1)
                if abs(self.player.car.rotation[2]) < 10000:
                    self.output_vector[3] = -1
                else:
                    self.output_vector[3] = 0
            elif self.current_stage_index == 3:
                # drive to ball
                _time, steer = self.player.car.find_time_to_point(self.player.ball.position - np.array([0, -200, 0]))
                self.output_vector[1] = steer

                self.set_boost_if_not_near_max()

                distance_to_ball = self.player.ball.position - self.player.car.position
                flip_threshold = 500
                if distance_to_ball.dot(distance_to_ball) < flip_threshold ** 2:
                    self.increment_stage()
            elif self.current_stage_index == 4:
                # flip into ball
                self.output_vector[6] = 0
                self.do_flip(flip_pitch=-1, flip_yaw=0, boost=False)

        elif self.kickoff_label == 1 or self.kickoff_label == 3:
            # off-centre
            if self.current_stage_index == 0:
                centre_boost_point = np.array([0, -4245, 0])
                # set steer to turn left if on right (label=1), or turn right if on left (label=3)
                steer_magnitude = 0.15
                self.output_vector[1] = steer_magnitude if self.kickoff_label == 3 else -steer_magnitude

                # get centre boost
                # print(self.player.car.boost, self.stage_data.get('last_boost', 100))
                if self.stage_data.get('spawned', False) and self.player.car.boost > self.stage_data.get('last_boost',
                                                                                                         100):
                    self.increment_stage()
                if self.player.car.boost > 10:
                    self.stage_data['spawned'] = True
                self.stage_data['last_boost'] = self.player.car.boost
            elif self.current_stage_index == 1:
                # flip diagonally outward
                if self.kickoff_label == 1:
                    # off right kickoff - flip right
                    yaw = 1
                else:
                    # off left kickoff - flip left
                    yaw = -1
                self.do_flip(flip_pitch=-0.2, flip_yaw=yaw, boost=False)

            elif self.current_stage_index == 2:
                # drive to ball
                _time, steer = self.player.car.find_time_to_point(self.player.ball.position - np.array([0, -200, 0]))
                self.output_vector[1] = steer

                self.set_boost_if_not_near_max()

                distance_to_ball = self.player.ball.position - self.player.car.position
                flip_threshold = 500
                if distance_to_ball.dot(distance_to_ball) < flip_threshold ** 2:
                    self.increment_stage()
            elif self.current_stage_index == 3:
                # flip into ball
                self.output_vector[6] = 0
                self.do_flip(flip_pitch=-1, flip_yaw=0, boost=False)

        elif self.kickoff_label == 0 or self.kickoff_label == 4:
            # diagonal kickoff
            if self.current_stage_index == 0:
                # set steer to turn left if on right (label=1), or turn right if on left (label=3)
                steer_magnitude = 0.35
                self.output_vector[1] = steer_magnitude if self.kickoff_label == 4 else -steer_magnitude

                # get centre boost
                # print(self.player.car.boost, self.stage_data.get('last_boost', 100))
                if self.stage_data.get('spawned', False) and self.player.car.boost > self.stage_data.get('last_boost',
                                                                                                         100):
                    self.increment_stage()
                if self.player.car.boost > 10:
                    self.stage_data['spawned'] = True
                self.stage_data['last_boost'] = self.player.car.boost
            elif self.current_stage_index == 1:
                if self.kickoff_label == 0:
                    # off right kickoff - flip right
                    yaw = 1
                else:
                    yaw = -1
                self.do_flip(flip_pitch=-0.4, flip_yaw=yaw)

                if abs(self.player.car.rotation[2]) < 13000:
                    self.output_vector[3] = yaw
                elif abs(self.player.car.rotation[2]) > 20000:
                    self.output_vector[3] = -yaw
                else:
                    self.output_vector[3] = 0

            elif self.current_stage_index == 2:
                # drive to ball
                _time, steer = self.player.car.find_time_to_point(self.player.ball.position - np.array([0, -200, 0]))
                self.output_vector[1] = steer

                self.set_boost_if_not_near_max()

                distance_to_ball = self.player.ball.position - self.player.car.position
                flip_threshold = 500
                if distance_to_ball.dot(distance_to_ball) < flip_threshold ** 2:
                    self.increment_stage()
            elif self.current_stage_index == 3:
                # flip into ball
                self.output_vector[6] = 0
                if self.kickoff_label == 0:
                    # off right kickoff - flip right
                    yaw = 0.5
                else:
                    yaw = -0.5
                self.do_flip(flip_pitch=-1, flip_yaw=yaw, boost=False)
                if self.stage_data['flipped'] == 2:
                    self.output_vector[2] = -1

    def get_kickoff_label(self):
        # check if to do kickoff
        # find if closest player on team to ball
        team = self.player.car.team
        teammate_sqdistances = []

        for i in range(len(self.player.game_tick_packet.gamecars)):
            if i != self.player.car.index:
                _car = self.player.game_tick_packet.gamecars[i]
                if _car.Team == team:
                    _car_displacement = np.array(
                        (_car.Location.X, _car.Location.Y, _car.Location.Z)) - self.player.ball.position
                    teammate_sqdistances.append(_car_displacement.dot(_car_displacement))

        if teammate_sqdistances:
            # there are teammates
            # TODO: Deal with teammates. assign self.lock=0, self.urgency=0 if not doing kickoff
            pass

        else:
            # no teammates
            do_kickoff = True

        kickoff_label = None
        for i in range(len(self.kickoff_positions)):
            displacement = self.player.car.position[:2] - self.kickoff_positions[i]
            if displacement.dot(displacement) < 100:
                kickoff_label = i
        print('Kickoff Label:', kickoff_label)

        return kickoff_label

    def get_stages(self):
        if self.kickoff_label == 2:
            # centre
            return ['get boost', 'turn right slightly', 'flip diagonal left', 'drive to ball', 'flip into ball']
        elif self.kickoff_label == 1 or self.kickoff_label == 3:
            # off-centre
            return ['get boost', 'flip diagonal outwards', 'drive to ball', 'flip into ball']
        elif self.kickoff_label == 0 or self.kickoff_label == 4:
            # diagonal
            return ['get boost', 'flip diagonal outwards', 'drive to ball', 'flip into ball']

    def on_complete(self):
        # set urgency to 0 on completion
        super().on_complete()
        self.urgency = 0


class GetBoost(UtilityAction):
    def __init__(self, player):
        super().__init__(self, player)

    def update_output_vector(self):

        # find 3 nearest boosts
        car_position = self.player.car.position
        full_boost_positions = self.player.stadium.boost_info[100]['positions']
        displacements = full_boost_positions - car_position
        distances_to_boosts = (displacements * displacements).sum(axis=1)

        sorted_indices = np.argsort(distances_to_boosts)

        nearest_boost_position = None
        i = 0
        while i < len(sorted_indices) and nearest_boost_position is None:
            _boost_index = sorted_indices[i]

            if self.player.stadium.boost_info["is_active"][_boost_index]:
                nearest_boost_position = full_boost_positions[_boost_index]
            i += 1

        nearest_boost_position = full_boost_positions[np.argmin(distances_to_boosts)]

        self.player.car.find_time_to_point(nearest_boost_position, use_boost=True)

        # TODO: Complete this function


class ATBA(AttackAction):

    def __init__(self, player):
        super().__init__(player)
        self.urgency = 1
        self.output_vector = [1, 0, 0, 0, 0, 0, 0, 0, 0]  # throttle steer pitch yaw roll jump boost handbrake

    def update_output_vector(self):
        _time, steer = self.player.car.find_time_to_point(self.player.ball.position, use_boost=False)

        self.output_vector[1] = steer


class Hit(UtilityAction):
    def find_intercept_point(self, use_boost=False, leeway=0.4):
        # find time: ball position table
        t, x_v = self.player.ball.predict_ball_positions()
        # find position where time taken to reach < time - leeway
        check_every_xth = 5
        for i in range(len(t)):
            if i % check_every_xth != 1:
                continue
            ball_position = x_v[i, 0:3]
            time_to_ball, steer = self.player.car.find_time_to_point(ball_position, use_boost=use_boost)
            if time_to_ball + leeway < t[i]:
                return time_to_ball, ball_position
        print("CANNOT FIND INTERCEPT POINT")


class Shoot(AttackAction):

    def __init__(self, player):
        super().__init__(player)
        self.output_vector = [1, 0, 0, 0, 0, 0, 0, 0, 0]
