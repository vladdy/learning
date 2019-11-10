import numpy as np


# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with

    if Rover.nav_angles is not None:
        if Rover.picking_up == 1:
            Rover.throttle = 0
            Rover.steer = 0
            Rover.brake = Rover.brake_set
            Rover.mode = 'stop'
        elif Rover.near_sample == 1:
            Rover.brake = Rover.brake_set
            Rover.mode = 'stop'
            Rover.steer = 0
            Rover.send_pickup = True
            Rover.samples_collected += 1
        elif len(Rover.rock_dists) > 0:
            Rover.mode = 'rock'
            if Rover.vel < Rover.max_vel:
                Rover.throttle = Rover.throttle_set
            else:
                Rover.throttle = 0
            Rover.brake = 0
            Rover.steer = np.clip(np.mean(Rover.rock_angles * 180 / np.pi), -15, 15)
        elif Rover.mode == 'forward':
            if len(Rover.nav_angles) >= Rover.stop_forward:
                if Rover.vel < Rover.max_vel:
                    Rover.throttle = Rover.throttle_set
                else:
                    Rover.throttle = 0
                Rover.brake = 0
                Rover.steer = np.clip((np.mean(Rover.nav_angles * 180 / np.pi)), -15, 15)
            elif len(Rover.nav_angles) < Rover.stop_forward:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
                Rover.mode = 'stop'
        elif Rover.mode == 'stop':
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            elif Rover.vel <= 0.2:
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    Rover.brake = 0
                    Rover.steer = -15
                    Rover.throttle = -0.5
                if len(Rover.nav_angles) >= Rover.go_forward:
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180 / np.pi), -15, 15)
                    Rover.mode = 'forward'
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover

