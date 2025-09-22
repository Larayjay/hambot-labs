"""new_controller controller."""
from fairis_tools.my_robot import MyRobot
import os
import math

# path to FAIRIS-Lite/WebotsSim/worlds/Fall25/maze1.xml
#used chatgpt to help with maze set up 
HERE = os.path.dirname(__file__)
maze_path = os.path.normpath(os.path.join(HERE, '..', '..', 'worlds', 'Fall25', 'maze1.xml'))

robot = MyRobot()
robot.load_environment(maze_path)
robot.move_to_start()
print("Maze path:", maze_path)

WHEEL_RADIUS = 0.045  
WHEEL_DISTANCE = 0.092

def go_straight(x1, y1, x2, y2):
    # finding distance
    if x1 != x2:
        calc_distance = abs(x2 - x1)
    else:
        calc_distance = abs(y2 - y1)

    # record starting encoder values
    start_left, start_right = robot.get_encoder_readings()

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        left, right = robot.get_encoder_readings()
        # average distance traveled by both wheels
        distance = ((left - start_left) + (right - start_right)) / 2.0 * WHEEL_RADIUS

        if distance >= calc_distance - 0.02:
            robot.stop()
            return
        else:
            robot.set_left_motor_velocity(15)
            robot.set_right_motor_velocity(15)

def turn_to(target_heading):
    """
    Turn robot in place to absolute heading (0=East, 90=North, 180=West, 270=South).
    Uses compass + wheels. Angles in degrees.
    """
    # read current heading (degrees from East, CCW)
    current = robot.get_compass_reading()

    # compute shortest turn error (-180, 180]
    error = target_heading - current
    if error > 180:
        error -= 360
    elif error <= -180:
        error += 360

    # decide direction
    direction = 1 if error > 0 else -1   # + = CCW (left), - = CW (right)

    # turn until aligned within a small tolerance
    t0 = robot.experiment_supervisor.getTime()
    while robot.experiment_supervisor.step(robot.timestep) != -1:
        current = robot.get_compass_reading()
        err = target_heading - current
        if err > 180: err -= 360
        elif err <= -180: err += 360

        # stop if within tolerance
        if abs(err) < 2:   # 2 degrees tolerance
            robot.stop()
            return

        # turn in place: left wheel opposite of right wheel
        speed = 2.0 * direction   # rad/s, tune this
        robot.set_left_motor_velocity(-speed)
        robot.set_right_motor_velocity(speed)

#point 0 to point 1 
go_straight(2.0,-2.0,-1.5,-2.0)


#point 1 to point 2
t0 = robot.experiment_supervisor.getTime()

while robot.experiment_supervisor.step(robot.timestep) != -1:
    t = robot.experiment_supervisor.getTime() - t0  # elapsed just for the arc

    if t >= 1.378:
        robot.stop()
        robot.set_left_motor_velocity(0)
        robot.set_right_motor_velocity(0)
        break

    robot.set_left_motor_velocity(15)
    robot.set_right_motor_velocity(10.338)

turn_to(90)

#point 2 to 3
go_straight(-2.0,-1.5,-2.0,-0.5)

#point 3 4
t0 = robot.experiment_supervisor.getTime()
while robot.experiment_supervisor.step(robot.timestep) != -1:
    t = robot.experiment_supervisor.getTime() - t0  # elapsed just for the arc

    if t >= 2.755:
        robot.stop()
        robot.set_left_motor_velocity(0)
        robot.set_right_motor_velocity(0)
        break

    robot.set_left_motor_velocity(15)
    robot.set_right_motor_velocity(10.338)

#point 4 to 5
turn_to(315) #275+45
go_straight(-1.0,-0.5,-0.5,-1.0)

#point 5 to 6 
turn_to(0)
go_straight(-0.5,-1.0,2.0,-1.0)

#point 6 to 7
turn_to(90)
go_straight(2.0,-1.0,2.0,0.0)

#point 7 to 8 
turn_to(180)
go_straight(2.0,0.0,0.0,0.0)

#point 8 to 9
turn_to(90)
go_straight(0.0,0.0,1.0,0.0)

#point 9 to 10
turn_to(180)
go_straight(0.0, 1.0,-2.0,1.0)

#point 10 to 11
# --- P10 -> P11 (two-arc, no spins) ---
# Arc 1 (clockwise)
turn_to(90)
t0 = robot.experiment_supervisor.getTime()
while robot.experiment_supervisor.step(robot.timestep) != -1:
    t = robot.experiment_supervisor.getTime() - t0
    if t >= 4.649:
        robot.set_left_motor_velocity(0.0)
        robot.set_right_motor_velocity(0.0)
        break
    robot.set_left_motor_velocity(15.000)    # rad/s
    robot.set_right_motor_velocity(11.546)   # rad/s

#point 11 to 12
turn_to(0)
go_straight(-1.0,2.0,1.5,2.0)

