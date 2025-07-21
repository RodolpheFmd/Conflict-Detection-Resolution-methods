#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import random

def main():
    approach = 'grid'#, 'general'
    sim = Simulation(approach)

    for t in range(sim.steps):
        sim.run(t)

    sim.plot()

# Force encoding : linear, no fp, and also compare horiwontal traje to ignore the deconfliction.

class Simulation:
    def __init__(self, approach, tsim=20, dt=0.05, n_vehicles=6):
        self.approach = approach
        self.dt = dt                        # Time step (s)
        self.tsim = tsim
        self.steps = int(self.tsim/self.dt) # Wall simulation time (s)
        self.n_vehicles = n_vehicles

        self.epsilon = 1e-6
        self.colors = ['b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y', 'b','g','r','c','m','y']

        self.airspace = Airspace(self.n_vehicles, self.approach)
        self.solver = ControlBarrier()
        self.traffic = Traffic(self.steps, n_vehicles)

        self.idx = 0

    def run(self, t):

        self.traffic.update(self.airspace.flight_plans, t)

        self.v_new = np.zeros(self.n_vehicles)

        #self.grid_approach(t)
        if self.approach == 'grid':
            self.step_grid(t)
        else:
            self.step(t)
        
        self.traffic.priority += 1 - self.v_new
        self.traffic.pos += self.v_new * self.dt
        self.traffic.v_prev = self.v_new

    def step(self, t):
        for ownship_idx in range(self.n_vehicles):
            pi = self.traffic.pos_hist[ownship_idx, t]
            LB, UB = -np.inf, np.inf
            for intruder_idx in range(self.n_vehicles):
                if ownship_idx == intruder_idx:
                    continue

                pj = self.traffic.pos_hist[intruder_idx, t]
                diff = pi - pj
                h_val = diff.dot(diff) - self.solver.los**2
                grad_h = 2 * diff

                dir_i = self.airspace.flight_plans['paths'][ownship_idx](self.traffic.pos[ownship_idx] + self.epsilon) - pi
                dir_j = self.airspace.flight_plans['paths'][intruder_idx](self.traffic.pos[intruder_idx] + self.epsilon) - pj

                # Normalize directions to avoid influence from path scale
                dir_i /= np.linalg.norm(dir_i) + self.epsilon
                dir_j /= np.linalg.norm(dir_j) + self.epsilon

                Lf_h = -grad_h.dot(dir_j) * self.traffic.v_prev[intruder_idx]
                a = grad_h.dot(dir_i)
                b = -(Lf_h + self.solver.alpha * h_val)

                if abs(a) < self.epsilon:
                    continue
                if a > 0:
                    LB = max(LB, b / a)
                else:
                    UB = min(UB, b / a)

            v_i = np.clip(self.traffic.ini_spd[ownship_idx], LB, UB)
            self.v_new[ownship_idx] = max(0.0, v_i)

    def step_grid(self, t):
        for ownship_idx in range(self.n_vehicles):
            LB, UB = -np.inf, np.inf
            pi = self.traffic.pos_hist[ownship_idx, t]
            for intruder_idx in range(self.n_vehicles):
                if ownship_idx == intruder_idx:
                    continue
                horizontal_own = (ownship_idx < int(self.n_vehicles/2))
                horizontal_int = (intruder_idx < int(self.n_vehicles/2))
                if horizontal_own == horizontal_int:
                    continue
                if horizontal_own:
                    y_own = self.airspace.flight_plans['latitude'][ownship_idx]
                    x_int = self.airspace.flight_plans['longitude'][intruder_idx-int(self.n_vehicles/2)]
                    s_i_int = x_int + self.airspace.flight_plans['path length']/2
                    s_j_int = y_own + self.airspace.flight_plans['path length']/2
                else:
                    x_own = self.airspace.flight_plans['longitude'][ownship_idx-int(self.n_vehicles/2)]; y_int = self.airspace.flight_plans['latitude'][intruder_idx]
                    s_i_int = y_int + self.airspace.flight_plans['path length']/2
                    s_j_int = x_own + self.airspace.flight_plans['path length']/2

                t_i = (s_i_int - self.traffic.pos[ownship_idx]) / self.traffic.v_prev[ownship_idx] if self.traffic.v_prev[ownship_idx] > self.epsilon else np.inf
                t_j = (s_j_int - self.traffic.pos[intruder_idx]) / self.traffic.v_prev[intruder_idx] if self.traffic.v_prev[intruder_idx] > self.epsilon else np.inf

                if (t_i > t_j) or (abs(t_i - t_j) < self.epsilon and ownship_idx > intruder_idx):

                    pj = self.traffic.pos_hist[intruder_idx, t]
                    diff = pi - pj
                    h_val = diff.dot(diff) - self.solver.los**2
                    grad_h = 2 * diff

                    dir_i = np.array([1.0, 0.0]) if horizontal_own else np.array([0.0, 1.0])
                    dir_j = np.array([1.0, 0.0]) if horizontal_int else np.array([0.0, 1.0])

                    Lf_h = -grad_h.dot(dir_j) * self.traffic.v_prev[intruder_idx]
                    a = grad_h.dot(dir_i)
                    b = -(Lf_h + self.solver.alpha * h_val)
                    if abs(a) < self.epsilon:
                        continue
                    if a > 0:
                        LB = max(LB, b / a)
                    else:
                        UB = min(UB, b / a)

            v_i = np.clip(self.traffic.ini_spd[ownship_idx], LB, UB)
            self.v_new[ownship_idx] = max(0.0, v_i)
    
    def plot(self):

        fig, ax = plt.subplots(figsize=(6,6))
        
        if self.approach == 'grid':
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

        else:
            ax.set_xlim(self.airspace.xmin, self.airspace.xmax)
            ax.set_ylim(self.airspace.ymin, self.airspace.ymax)
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_title("CBF Conflict Resolution")

            for i in range(self.n_vehicles):
                path_fn = self.airspace.flight_plans['paths'][i]
                s_vals = np.linspace(0, self.traffic.ini_spd[i] * self.tsim, 100)
                traj = np.array([path_fn(s) for s in s_vals])
                ax.plot(traj[:, 0], traj[:, 1], 'k--', alpha=0.3)

            dots = []
            for i in range(self.n_vehicles):
                dot, = ax.plot([], [], f'{self.colors[i]}o', label=f'Vehicle {i + 1}')
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
    def __init__(self, steps, n_vehicles):

        self.steps = steps
        self.n_vehicles = n_vehicles
        self.priority =  np.zeros(self.n_vehicles)
        self.ini_spd = np.random.uniform(1, 5, size=(self.n_vehicles))

        self.pos = np.zeros(self.n_vehicles)                       # path parameters
        self.v_prev = self.ini_spd

        self.pos_hist = np.zeros((self.n_vehicles, self.steps, 2)) # record positions

    def update(self, flight_plans, time):
        for ac in range(self.n_vehicles):
            self.pos_hist[ac, time] = flight_plans['paths'][ac](self.pos[ac])

class Airspace:
    def __init__(self, n_vehicles, approach, corridor_sep = 3.0, box=7):
        self.n_vehicles = n_vehicles
        self.corridor_sep = corridor_sep
        self.box = box

        self.path_length = corridor_sep * (int(self.n_vehicles/2)+1)
        
        if approach == 'grid':
            self.generate_grid_flight_plans
        else:
            self.generate_random_flight_plans

    @property
    def generate_random_flight_plans(self):

        origins = np.random.uniform(0, self.box, size=(self.n_vehicles, 2))
        heading = np.random.uniform(0, 2*np.pi, size=(self.n_vehicles, 1))
        
        distances = np.random.uniform(8, 12, size=(self.n_vehicles,1))

        directions = np.concatenate((np.cos(heading), np.sin(heading)), axis=1)
        destinations = origins + directions * distances

        def make_path_fn(origin, direction):
            return lambda s: origin + s * direction

        paths = [make_path_fn(o, d) for o, d in zip(origins, directions)]

        all_points = np.vstack([origins, destinations])
        self.xmin, self.ymin = np.min(all_points, axis=0) - 0.2
        self.xmax, self.ymax = np.max(all_points, axis=0) + 0.2

        print(f'paths: {paths}')

        self.flight_plans = {'origins': origins,
                             'destinations': destinations,
                             'heading': heading,
                             'distance': distances,
                             'paths': paths}

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