import numpy as np
from quicktracer import trace
import actions.car_actions as a


class Player:

    def __init__(self, car, stadium, ball, defensive_focus=1, offensive_focus=1):
        self.car = car
        self.stadium = stadium
        self.ball = ball
        self.current_action = None
        self.game_tick_packet = None

        self.car.player = self
        self.ball.player = self

        self.defensive_focus = defensive_focus
        self.offensive_focus = offensive_focus

        self.actions = self.create_actions()
        self.last_action = None
        self.waiting_for_actions_reset = False

    def create_actions(self):
        """Creates new action list as reset after each goal scored."""
        print("Creating actions")
        actions = [a.KickOff(self), a.ATBA(self)]
        return actions

    def update(self, game_tick_packet):
        self.game_tick_packet = game_tick_packet
        self.car.update(game_tick_packet)
        self.ball.update(game_tick_packet)
        self.stadium.update(game_tick_packet)

        # create new list of actions if game_tick_packet.GameInfo.bRoundActive turns false
        if game_tick_packet.gameInfo.bRoundActive is False and self.waiting_for_actions_reset:
            self.actions = self.create_actions()
            self.waiting_for_actions_reset = False
        elif game_tick_packet.gameInfo.bRoundActive is True:
            self.waiting_for_actions_reset = True

    def steer(self):
        # find ball trajectory
        # point = np.array([0, 0, 0])
        point = self.ball.position - np.array([0, 100, 0])
        time, steer = self.car.find_time_to_point(point)
        trace(time)
        trace(steer)

        return [1, steer, 0, 0, 0, 0, 0, 0]

    def find_action(self):
        for action in self.actions:
            action.update_urgency()

        current_locked_actions = []
        for action in self.actions:
            if action.lock:
                current_locked_actions.append(action)
        if len(current_locked_actions) > 1:
            print("Multiple locked actions! Using first.")

        if current_locked_actions:
            chosen_action = current_locked_actions[0]
        else:
            # find urgency of actions
            sorted_list = sorted(self.actions, key=lambda x: x.urgency, reverse=True)
            chosen_action = sorted_list[0]

        if chosen_action != self.last_action:
            print("Found new action: %s (Old: %s)" % (chosen_action, self.last_action))
            self.last_action = chosen_action

        return chosen_action

    def get_output_vector(self):

        action = self.find_action()
        action.update_output_vector()

        output_vector = action.output_vector
        self.log_output_vector(output_vector)

        self.test_hit()
        return output_vector

    def log_output_vector(self, output_vector):
        self.car.last_steers.append(output_vector[1])


    def test_hit(self):
        try:
            print('Hit time: %s, position: %s' % self.hit.find_intercept_point())
        except AttributeError:
            self.hit = a.Hit(self)
        except TypeError:
            pass

