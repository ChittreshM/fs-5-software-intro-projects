import matplotlib.pyplot as plt
import numpy as np
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

'''
PID Tuning:
critical gain value (Ku) = 25
ultimate osciallation period (Tu) = 0.32

Kp = 0.6 * Ku
Ki = (1.2 * Ku) / Tu
Kd = 0.075 * Ku * Tu

'''

'''
K_P = 3 # when kP is too high, then the velocity oscillates around the desired velocity, and there's a sharper increase to the final velocity instead of having smooth acceleration
K_I = 0.02
K_D = 0
'''

CRUISE_GAINS = (0.3, 0.02, 0.0)   # (K_P, K_I, K_D) gentle: smooth acceleration
BRAKE_GAINS  = (3.0, 0.02, 0.0)   # aggressive: tracks the braking curve tightly

# when the desired velocity is lower, using just kP shows that the car reaches desired velocity but starts oscillating and decreases slightly
car = make_car(desired_v=20.0, dt=0.1)
 
STEPS = 1000

# Extension 1 (stopping point): low K_P (0.3) braked too late and overshot to ~335 m.
# Higher K_P (3) tracks the braking curve closely; tiny K_I (0.02) limits windup.
MAX_VELOCITY = car['desired_v']
FINAL_X = 300.0
BRAKE_DECELERATION = 3.0

#WRITE CODE HERE
velocities = []
errors = []
times = []
positions = []
desired_vs = []

for i in range(STEPS):

    # Stopping point: lower the target speed as we approach FINAL_X
    remaining_distance = max(FINAL_X - car['x'], 0)
    car['desired_v'] = min(MAX_VELOCITY, (np.sqrt(2 * remaining_distance * BRAKE_DECELERATION)))

     # Gain scheduling with blending (bumpless transfer):
    # blend = 0 while cruising, rises to 1 as the target drops to 0,
    # so the gains slide gradually from cruise to brake instead of jumping.
    blend = min(1, 3 * (1 - car['desired_v'] / MAX_VELOCITY))
    K_P = CRUISE_GAINS[0] + blend * (BRAKE_GAINS[0] - CRUISE_GAINS[0])
    K_I = CRUISE_GAINS[1] + blend * (BRAKE_GAINS[1] - CRUISE_GAINS[1])
    K_D = CRUISE_GAINS[2] + blend * (BRAKE_GAINS[2] - CRUISE_GAINS[2])

    desired_acceleration, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    throttle_percentage = acceleration_to_throttle_percentage(desired_acceleration)

    update(car, throttle_percentage)

    velocities.append(car['v'])
    errors.append(error)
    times.append(car['t'])
    positions.append(car['x'])
    desired_vs.append(car['desired_v'])

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

ax1.plot(times, velocities, label='velocity')
ax1.plot(times, desired_vs, '--', color='red', label='desired velocity')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Velocity over Time')
ax1.legend()

ax2.plot(times, errors)
ax2.axhline(0, color = 'gray', linestyle = '--')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (m/s)')
ax2.set_title('Error over Time')

fig2, ax = plt.subplots()
ax.plot(positions, velocities, label="velocity")
ax.plot(positions, desired_vs, "--", label="desired velocity")
ax.axvline(FINAL_X, color="red", linestyle="--", label="stop point")  # vertical line
ax.set_xlabel("Distance (m)")
ax.set_ylabel("Velocity (m/s)")
ax.set_title("Velocity over Distance")
ax.legend()

plt.tight_layout()
plt.show()