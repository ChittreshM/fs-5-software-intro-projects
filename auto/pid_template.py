import numpy as np


def make_car(desired_v:float=20.0, dt:float=0.1) -> dict:
    """ 
    Generates a dictionary that holds all the car's values. Keeps track of state varaibles.
    """
    car_state_dictionary : dict[str, float] = {
        "v" : 0, #velocity of your car 
        "a" : 0, #acceleration of your car
        "t" : 0, #time of your car
        "x" : 0, #position of your car
        "dt" : dt, #time step of your car, how much the time changes every time you update/step
        "desired_v" : desired_v, #desired velocity of your car, the velocity you want to maintain
        "step" : 0,
    
        #hint: use these variables in the integral and derivative portion of your PID control (steps 5 and 6 )
        "error_prev" : None,
        "net_integral" : 0.0
    }
    return car_state_dictionary

def update(car: dict, throttle_perc: float, mass: float = 1000, max_throttle_force: float = 5000, friction: float = 2.0) -> None:
        """
        Updates the car's state variables based on the throttle percentage.
        Use this function after finding throttle percentage to update the car's state variables.

        Inputs:
        car: dictionary containing the car's state variables
        throttle_perc: float, throttle percentage (-1 to 1)

        Outputs:
        None, but updates the car's state variables
        """
        force = throttle_perc * max_throttle_force
        car["a"] = (force / mass) - friction
        car["v"] += car["a"] * car["dt"]
        car["x"] += car["v"] * car["dt"]
        car["t"] += car["dt"]
        car["step"] += 1


def calculate_desired_acceleration(car: dict, K_P: float, K_I: float = 0.0, K_D: float = 0.0) -> tuple[float, float]:
        #input: car["v"], car["desired_v"] (floats)
        #output: desired acceleration and error tuple(float, float)

        """
        PID controller: calculates how much acceleration the car needs to reach its desired velocity.
        Combines the proportional (current error), integral (accumulated error),
        and derivative (rate of change of error) terms.

        Inputs:
        car: dictionary containing the car's state variables (uses v, desired_v, dt,
             net_integral, error_prev, and updates net_integral and error_prev)
        K_P: float, proportional gain
        K_I: float, integral gain (default 0, which turns the I term off)
        K_D: float, derivative gain (default 0, which turns the D term off)

        Outputs:
        tuple(float, float): (desired acceleration in m/s^2, error in m/s)
        """

        error = car['desired_v'] - car['v']

        error_integral = error * car['dt']
        car['net_integral'] += error_integral

        if car['error_prev'] is None:
               error_derivative = 0.0
        else:
                error_derivative = (error - car['error_prev'])/(car['dt'])
        car['error_prev'] = error

        desired_acceleration = (K_P * error) + (K_I * car['net_integral']) + (K_D * error_derivative)

        return desired_acceleration, error




def acceleration_to_throttle_percentage(acceleration_desired: float, mass: float = 1000, max_throttle_force: float = 5000) -> float:
        #input: desired_acceleration(float)
        #output: throttle percentage (float, -1 to 1)

        """
        Converts a desired acceleration into a throttle percentage for the motor,
        using Newton's second law (a = F / m) to find the max possible acceleration.

        Inputs:
        acceleration_desired: float, acceleration the controller wants (m/s^2)
        mass: float, mass of the car (kg). Must match the mass used in update()
        max_throttle_force: float, motor force at 100% throttle (N). Must match update()

        Outputs:
        float, throttle percentage clipped to -1 to 1 (-100% to 100%)
        """
        
        max_acceleration = max_throttle_force / mass  # check difference between force and max_throttle_force
        throttle_perc = acceleration_desired / max_acceleration # check why it's acceleration_desired here but desired_acceleration in previous function
        true_throttle_perc = float(np.clip(throttle_perc, -1, 1))

        return true_throttle_perc
