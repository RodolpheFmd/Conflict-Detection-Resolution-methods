#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

def main():
    sim = Simulation()

    for t in range(sim.steps):
        sim.run(t)

    sim.plot()

class Simulation:
    def __init__(self, tsim=20, dt=0.05, n_vehicles=6):
        self.dt = dt                        # Time step (s)
        self.tsim = tsim
        self.steps = int(self.tsim/self.dt) # Wall simulation time (s)
        self.n_vehicles = n_vehicles

        self.epsilon = 1e-6
        self.colors = ['b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y']

        self.airspace = Airspace(self.n_vehicles)
        self.solver = ControlBarrier()
        self.traffic = Traffic(self.steps, n_vehicles)

    def run(self, t):

        self.traffic.update(self.airspace.flight_plans, t)

        v_new = np.zeros(self.n_vehicles)
        for i in range(self.n_vehicles):
            LB, UB = -np.inf, np.inf
            pi = self.traffic.pos_hist[i, t]
            for j in range(self.n_vehicles):
                if i == j:
                    continue
                horizontal_i = (i < int(self.n_vehicles/2))
                horizontal_j = (j < int(self.n_vehicles/2))
                if horizontal_i == horizontal_j:
                    continue
                if horizontal_i:

                    y_i = self.airspace.flight_plans['latitude'][i]; x_j = self.airspace.flight_plans['longitude'][j-int(self.n_vehicles/2)]
                    s_i_int = x_j + self.airspace.flight_plans['path length']/2
                    s_j_int = y_i + self.airspace.flight_plans['path length']/2
                else:

                    x_i = self.airspace.flight_plans['longitude'][i-int(self.n_vehicles/2)]; y_j = self.airspace.flight_plans['latitude'][j]
                    s_i_int = y_j + self.airspace.flight_plans['path length']/2
                    s_j_int = x_i + self.airspace.flight_plans['path length']/2

                t_i = (s_i_int - self.traffic.s[i]) / self.traffic.v_prev[i] if self.traffic.v_prev[i] > self.epsilon else np.inf
                t_j = (s_j_int - self.traffic.s[j]) / self.traffic.v_prev[j] if self.traffic.v_prev[j] > self.epsilon else np.inf

                if (t_i > t_j) or (abs(t_i - t_j) < self.epsilon and i > j):

                    pj = self.traffic.pos_hist[j, t]
                    diff = pi - pj
                    h_val = diff.dot(diff) - self.solver.los**2
                    grad_h = 2 * diff

                    dir_i = np.array([1.0, 0.0]) if horizontal_i else np.array([0.0, 1.0])
                    dir_j = np.array([1.0, 0.0]) if horizontal_j else np.array([0.0, 1.0])

                    Lf_h = -grad_h.dot(dir_j) * self.traffic.v_prev[j]
                    a = grad_h.dot(dir_i)
                    b = -(Lf_h + self.solver.alpha * h_val)
                    if abs(a) < self.epsilon:
                        continue
                    if a > 0:
                        LB = max(LB, b / a)
                    else:
                        UB = min(UB, b / a)

            v_i = np.clip(self.traffic.ini_spd, LB, UB)
            v_new[i] = max(0.0, v_i)

        self.traffic.s += v_new * self.dt
        self.traffic.v_prev = v_new

    def plot(self):
        fig, ax = plt.subplots(figsize=(6,6))
        ax.set_xlim(-self.airspace.flight_plans['path length']/2 - 4, self.airspace.flight_plans['path length']/2 + 4)
        ax.set_ylim(-self.airspace.flight_plans['path length']/2 - 4, self.airspace.flight_plans['path length']/2 + 4)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_title("CBF Conflict Resolution")

        t_line = np.linspace(-self.airspace.flight_plans['path length']/2, self.airspace.flight_plans['path length']/2, 200)
        for y in self.airspace.flight_plans['latitude']:
            ax.plot(t_line, y*np.ones_like(t_line), 'k--', alpha=0.5)
        for x in self.airspace.flight_plans['longitude']:
            ax.plot(x*np.ones_like(t_line), t_line, 'k--', alpha=0.5)

        dots = []

        for i in range(self.n_vehicles):
            dot, = ax.plot([], [], f'{self.colors[i]}o', label=f'Vehicle {i+1}')
            dots.append(dot)
        ax.legend(loc='upper right')

        def update(frame):
            for i, dot in enumerate(dots):
                dot.set_data([self.traffic.pos_hist[i, frame, 0]], [self.traffic.pos_hist[i, frame, 1]])
            return dots

        ani = FuncAnimation(fig, update, frames=self.steps, interval=50, blit=True)

        plt.show(block=False)
        plt.pause(self.tsim + 1)
        plt.close(fig)

class ControlBarrier:
    def __init__(self, los=1.0, alpha  = 1.0):
        self.los = los                 # Sector center, x coordinate.
        self.alpha = alpha             # CBF gain

class Traffic:
    def __init__(self, steps, n_vehicles, ini_spd = 1.0):
        
        self.ini_spd = ini_spd
        self.steps = steps
        self.n_vehicles = n_vehicles

        self.s = np.zeros(self.n_vehicles)                    # path parameters
        self.v_prev = np.ones(self.n_vehicles) * self.ini_spd # previous-step speeds
        self.pos_hist = np.zeros((self.n_vehicles, self.steps, 2))    # record positions

    def update(self, flight_plans, time):
        for i in range(self.n_vehicles):
            self.pos_hist[i, time] = flight_plans['paths'][i](self.s[i])

class Airspace:
    def __init__(self, n_vehicles, corridor_sep = 3.0):
        self.n_vehicles = n_vehicles
        self.corridor_sep = corridor_sep

        self.path_length = corridor_sep * (int(self.n_vehicles/2)+1)
        
        self.generate_grid_flight_plans

    @property
    def generate_grid_flight_plans(self):

        coordinates = np.array([idx*self.corridor_sep for idx in range(int(self.n_vehicles/2))])
        self.flight_plans = {'latitude': coordinates - coordinates[-1]/2, 
                             'longitude': coordinates - coordinates[-1]/2, 
                             'path length': self.path_length}
        
        self.flight_plans.update({'paths': self.generate_paths})  

    @property
    def generate_paths(self):
        return [lambda s, y=y: np.array([s - self.flight_plans['path length']/2, y]) for y in self.flight_plans['latitude']] + [lambda s, x=x: np.array([x, s - self.flight_plans['path length']/2]) for x in self.flight_plans['longitude']]

if __name__ == "__main__":
    main()