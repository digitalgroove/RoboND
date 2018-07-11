import numpy as np
from math import degrees
from math import atan2
from math import pi, floor
import math

def angle_to(p1, p2, robotHeading):
    angle = atan2(p2[1] - p1[1], p2[0] - p1[0]) - math.radians(robotHeading)

    # Next calculate shortest turn, to returns value in range [-Pi,+Pi]
    # custom modulo calc. because fmod return has same sign as dividend
    #angle = (angle + math.pi) - floor((angle + math.pi)/(2 * math.pi)) * (2 * math.pi)
    a = math.fmod(angle + math.pi, 2 * math.pi)
    if a >= 0:
        return degrees(a - math.pi)
    else:
        return degrees(a + math.pi)


def HeadingToRock(wpt_x, wpt_y, robotHeading):
    # calculate angle towards goal, result in range [-2*Pi,+2*Pi]
    angle = atan2(wpt_y, wpt_x) - robotHeading #RHCS compliant
    # calculate shortest turn, returns value in range [-Pi,+Pi]
    # custom modulo calc. because fmod return has same sign as dividend
    a = fmod(angle + math.pi, 2 * math.pi)
    if a >= 0:
        return a - math.pi
    else:
        return a + math.pi
    angle = (angle + math.pi) - math.floor((angle + math.pi)/(2 * math.pi)) * (2 * math.pi)
    return angle - math.pi

# if ((dest - source + 360) % 360 < 180)
#
# double normalize180(double angle)
# {
#     double a = fmod(angle + Pi, 2 * Pi);
#     return a >= 0 ? (a - Pi) : (a + Pi); // ?:ternary operator, inline if
# }


# This is where you can build a decision tree for determining throttle, brake and steer
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!

    # Example:
    # Check if we have vision data to make decisions with
    if Rover.mode == 'pick up':
        Rover.rock_angle = angle_to(Rover.pos, Rover.rock_list[0], Rover.yaw)
        first_break = False
        if Rover.vel > 0.8 and not first_break:
            Rover.throttle = 0
            Rover.brake = Rover.brake_set
            first_break = True
        elif Rover.vel > 0.8 and first_break:
            Rover.throttle = 0
        else:
            if (abs(Rover.rock_angle) >= 3):
                # turn in place
                Rover.comment = 'turn in place'
                Rover.brake = 0
                Rover.rock_angle = angle_to(Rover.pos, Rover.rock_list[0], Rover.yaw)
                Rover.steer = np.clip(Rover.rock_angle, -15, 15)
            else:
                Rover.comment = 'aligned'
                if not Rover.near_sample:
                    Rover.comment = 'aligned not near'
                    Rover.brake = 0
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    #Rover.rock_angle = angle_to(Rover.pos, Rover.rock_list[0], Rover.yaw)
                    #Rover.steer = np.clip(Rover.rock_angle, -15, 15)
                else: # Else break and pick up
                    Rover.comment = 'aligned and near'
                    Rover.throttle = 0
                    Rover.brake = Rover.brake_set
                    if not Rover.picking_up:
                        Rover.comment = 'I AM HERE!'
                        Rover.send_pickup = True
                        del Rover.rock_list[:]
                        Rover.rock_angle = None
                        #Rover.mode = 'forward'

        # if not Rover.near_sample:
        #     # If navigable terrain looks good
        #     # and velocity is below max, then throttle
        #     if Rover.vel > 1:
        #         # Hit the brakes!
        #         Rover.throttle = 0
        #         Rover.brake = Rover.brake_set
        #     elif Rover.vel < 1:
        #         # Set throttle value to throttle setting
        #         Rover.throttle = Rover.throttle_set
        #     else: # Else coast
        #         Rover.throttle = 0
        #     Rover.brake = 0
        #     # Set steering to average angle clipped to the range +/- 15
        #     Rover.steer = np.clip(np.mean(Rover.rock_angle * 180/np.pi), -15, 15)
        # If it is near sample go to 'stop' mode
        # else:
        #     # Set mode to "stop" and hit the brakes!
        #     Rover.throttle = 0
        #     # Set brake to stored brake value
        #     Rover.brake = Rover.brake_set
        #     Rover.steer = np.clip(np.mean(Rover.rock_angle * 180/np.pi), -15, 15)



    elif Rover.nav_angles is not None and Rover.mode == 'forward':
        # Check for Rover.mode status
        if Rover.mode == 'forward':
            # Check the extent of navigable terrain
            if len(Rover.nav_angles) >= Rover.stop_forward:
                # If mode is forward, navigable terrain looks good
                # and velocity is below max, then throttle
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.nav_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'

        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2:
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.nav_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
                    Rover.steer = -15 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.nav_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
    # Just to make the rover do something
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0

    # # If in a state where want to pickup a rock send pickup command
    # if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    #     Rover.send_pickup = True
    #     Rover.mode = 'forward'
    #     del Rover.rock_list[:]
    # elif Rover.near_sample and Rover.vel == 0 and Rover.picking_up:
    #     Rover.throttle = 0
    #     Rover.brake = Rover.brake_set
    #     Rover.steer = 0

    return Rover
