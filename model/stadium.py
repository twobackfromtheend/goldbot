import numpy as np


class Stadium:
    def __init__(self):
        self.side_wall_distance = 4096
        self.back_wall_distance = 5120
        self.ceiling_distance = 2044
        self.floor = 0

        self.boost_info = {100: {'positions': [], 'is_active': [], 'timer': []},
                           12: {'positions': [], 'is_active': [], 'timer': []}}
        self.boost_positions_set = False

        self.boost_radius = (208, 144)  # radii and height for full, then small boost
        self.boost_height = (168, 165)

    def update(self, game_tick_packet):
        if not self.boost_positions_set:
            self.set_boost_positions(game_tick_packet)

        for i in range(len(game_tick_packet.gameBoosts)):
            boost = game_tick_packet.gameBoosts[i]
            if i < 28:
                self.boost_info[12]['is_active'] = boost.bActive
                self.boost_info[12]['timer'] = boost.Timer
            else:
                self.boost_info[100]['is_active'] = boost.bActive
                self.boost_info[100]['timer'] = boost.Timer

    def set_boost_positions(self, game_tick_packet):
        boost_100_locations = []
        boost_12_locations = []
        for i in range(len(game_tick_packet.gameBoosts)):
            boost = game_tick_packet.gameBoosts[i]
            location = boost.Location.X, boost.Location.Y, boost.Location.Z

            # full boosts have indices 28-33
            if i < 28:
                boost_12_locations.append(location)
            else:
                boost_100_locations.append(location)

        self.boost_info[100]['positions'] = np.array(np.array(boost_100_locations))
        self.boost_info[12]['positions'] = np.array(np.array(boost_12_locations))
