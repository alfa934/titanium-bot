import numpy as np
import matplotlib.pyplot as plt

class MotionProfile:
    def __init__(self, max_acceleration, max_velocity, distance):
        self.max_accel = abs(max_acceleration)
        self.max_vel = abs(max_velocity)
        self.distance = distance
        self.sign = 1.0 if distance >= 0 else -1.0
        self.compute_profile_parameters()

    def compute_profile_parameters(self):
        abs_distance = abs(self.distance)

        if abs_distance < 1e-9:
            self.accel_time = 0
            self.cruise_time = 0
            self.total_time = 0
            self.accel_distance = 0
            self.cruise_distance = 0
            return

        self.accel_time = self.max_vel / self.max_accel
        self.accel_distance = 0.5 * self.max_accel * (self.accel_time ** 2)

        if 2 * self.accel_distance > abs_distance:
            self.accel_time = np.sqrt(abs_distance / self.max_accel)
            self.accel_distance = 0.5 * self.max_accel * (self.accel_time ** 2)
            self.max_vel = self.max_accel * self.accel_time

        self.cruise_distance = abs_distance - 2 * self.accel_distance
        self.cruise_time = self.cruise_distance / self.max_vel
        self.total_time = 2 * self.accel_time + self.cruise_time

    def calculate(self, elapsed_time):
        t = elapsed_time

        if t < 0:
            t = 0
        if t > self.total_time:
            t = self.total_time

        state = {'position': 0.0, 'velocity': 0.0, 'acceleration': 0.0}

        # Phase calculations
        if t < self.accel_time:
            # Acceleration phase
            state['position'] = 0.5 * self.max_accel * t ** 2
            state['velocity'] = self.max_accel * t
            state['acceleration'] = self.max_accel
        elif t < self.accel_time + self.cruise_time:
            # Cruise phase
            cruise_t = t - self.accel_time
            state['position'] = self.accel_distance + self.max_vel * cruise_t
            state['velocity'] = self.max_vel
            state['acceleration'] = 0.0
        elif t < self.total_time:
            # Deceleration phase
            decel_t = t - (self.accel_time + self.cruise_time)
            decel_vel = self.max_vel - self.max_accel * decel_t
            state['position'] = self.accel_distance + self.cruise_distance + (self.max_vel + decel_vel) * 0.5 * decel_t
            state['velocity'] = decel_vel
            state['acceleration'] = -self.max_accel
        else:
            # Final state (motion complete)
            state['position'] = abs(self.distance)
            state['velocity'] = 0.0
            state['acceleration'] = 0.0

        # Apply direction sign
        state['position'] *= self.sign
        state['velocity'] *= self.sign
        state['acceleration'] *= self.sign

        return state

    def get_total_time(self):
        return self.total_time


def plot_motion_profile(profile, time_step=0.01):
    total_time = profile.get_total_time()
    time_points = np.arange(0, total_time + time_step, time_step)
    
    positions = []
    velocities = []
    accelerations = []

    for t in time_points:
        state = profile.calculate(t)
        positions.append(state['position'])
        velocities.append(state['velocity'])
        accelerations.append(state['acceleration'])

    plt.figure(figsize=(10, 8))

    # Position plot
    plt.subplot(3, 1, 1)
    plt.plot(time_points, positions, label='Position', color='blue')
    plt.ylabel('Position (m)')
    plt.title('Trapezoidal Motion Profile')
    plt.grid(True)

    # Velocity plot
    plt.subplot(3, 1, 2)
    plt.plot(time_points, velocities, label='Velocity', color='green')
    plt.ylabel('Velocity (m/s)')
    plt.grid(True)

    # Acceleration plot
    plt.subplot(3, 1, 3)
    plt.plot(time_points, accelerations, label='Acceleration', color='red')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.grid(True)

    plt.tight_layout()
    plt.show()


# Example usage
if __name__ == "__main__":
    max_accel = 4.0    # m/s²
    max_vel = 4.0      # m/s
    distance = 10.0    # m

    profile = MotionProfile(max_accel, max_vel, distance)
    plot_motion_profile(profile)