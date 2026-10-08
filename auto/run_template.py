import matplotlib.pyplot as plt
from pid_template import make_car
from pid_template import update
from pid_template import calculate_desired_acceleration
from pid_template import acceleration_to_throttle_percentage

K_P = 10 # when kP is too high, then the velocity oscillates around the desired velocity, and there's a sharper increase to the final velocity instead of having smooth acceleration
K_I = 5
K_D = 0.0
 
STEPS = 550
 
car = make_car(desired_v=20.0, dt=0.1)

#WRITE CODE HERE
velocities = []
errors = []
times = []

for i in range(STEPS):
    desired_acceleration, error = calculate_desired_acceleration(car, K_P, K_I, K_D)
    throttle_percentage = acceleration_to_throttle_percentage(desired_acceleration)

    update(car, throttle_percentage)

    velocities.append(car['v'])
    errors.append(error)
    times.append(car['t'])

fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

ax1.plot(times, velocities, label='velocity')
ax1.axhline(car['desired_v'], color = 'red', linestyle = '--', label = 'desired velocity')
ax1.set_ylabel('Velocity (m/s)')
ax1.set_title('Velocity over Time')
ax1.legend()

ax2.plot(times, errors)
ax2.axhline(0, color = 'gray', linestyle = '--')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Error (m/s)')
ax2.set_title('Error over Time')

plt.tight_layout()
plt.show()