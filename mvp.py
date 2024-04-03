import numpy as np

aircraft = [{'name': 'X',
            'Position': {'latitude': 0,
                         'longitude': 0,
                         'altitude': 0,
                         'heading': 0,
                         'speed':0},
            'Priority': 0,
            'detestination': {'latitude': 0,
                              'longitude': 0,
                              'altitude': 0}}, #[...]
            {'name': 'Y',
             'Position': {'latitude': 0,
                          'longitude': 0,
                          'altitude': 0,
                          'heading': 0,
                          'speed':0},
             'Priority': 0,
             'detestination': {'latitude': 0,
                               'longitude': 0,
                               'altitude': 0}}]

class Geo:
    def __init__(self):
        self.r_earth = 6378137
        self.eccentricity2 = 0.00669437999014

    def geo_2_cartesian(self, position):

        lat_rad = np.radians(position['latitude'])
        lon_rad = np.radians(position['longitude'])

        N = self.r_earth / np.sqrt(1 - self.eccentricity2 * np.sin(lat_rad)**2)

        # Calculate Cartesian coordinates
        x = (N + position['altitude']) * np.cos(lat_rad) * np.cos(lon_rad)
        y = (N + position['altitude']) * np.cos(lat_rad) * np.sin(lon_rad)
        z = (N * (1 - self.eccentricity2) + position['altitude']) * np.sin(lat_rad)

        return np.array([x, y, z])

    def cartesian_2_geo(self, cart_position):

        r = np.sqrt(cart_position[0]**2 + cart_position[1]**2 + cart_position[2]**2)
        lat = np.arcsin(z / r)
        lon = np.arctan2(cart_position[1], cart_position[0])

        geo_position = {'latitude': np.degrees(lat),
                        'longitude': np.degrees(lon),
                        'altitude': r - self.r_earth}

        return geo_position

class MVP:
    def __init__(self):
        self.K = 8.9875517873681764 * 10**9
        self.geo = Geo()

    def compute_force(self, ownship, intruder):

        r_vec = np.array(self.geo.geo_2_cartesian(intruder['Position'])) - np.array(self.geo.geo_2_cartesian(ownship['Position']))
        r_mag = np.linalg.norm(r_vec)

        force_mag = self.K * intruder['Priority'] * ownship['Priority'] / r_mag**2

        return force_mag * r_vec / r_mag

    def adjust_trajectory(self, ownship, intruder, mass=1, time_step=1):

        pos = self.geo.geo_2_cartesian(ownship['Position'])

        force_vec = self.compute_force(ownship, intruder)

        # Calculate the acceleration vector
        acc_vec = force_vec / mass

        # Update the position based on the acceleration (simple kinematics)
        new_pos = pos + acc_vec * time_step**2

        return self.geo.cartesian_2_geo(new_pos)
