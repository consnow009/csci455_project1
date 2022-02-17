from controller import Robot, sys

# Global Time-step Constant
TIMESTEP = 64


# Controller function for robot
def exec_robot(robot):
    # this is specified in the documentation
    max_speed = 6.28 / 4

    # get the motors, enable, and set motor devices
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')

    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

    # get and enable all the proximity sensors (ps0-ps7)
    sensors = []
    for i in range(8):
        sensor_name = 'ps' + str(i)
        sensors.append(robot.getDevice(sensor_name))
        sensors[i].enable(TIMESTEP)

    # get and enable touch sensor
    touch = robot.getDevice('touch sensor')
    touch.enable(TIMESTEP)

    # main loop
    while robot.step(TIMESTEP) != -1:
        # print the sensor and its corresponding value
        for i in range(8):
            print("Sensor: {}, Value: {}".format(i, sensors[i].getValue()))

        # interpret prox sensor data
        right_wall = sensors[2].getValue() > 180
        front_wall = sensors[0].getValue() > 250

        # set initial speeds
        left_speed = max_speed
        right_speed = max_speed

        if front_wall or sensors[1].getValue() > 500:
            print("Rotate left")
            left_speed = -max_speed
            right_speed = max_speed
        else:
            if right_wall:
                print("Advance")
                left_speed = max_speed * 2
                right_speed = max_speed * 2
            else:
                print("Turn right")
                left_speed = max_speed * 3
                right_speed = 0
        # exit program execution once touch sensor trips
        print(touch.getValue())
        if touch.getValue() > 0:
            print("HIT")
            break

        # move the e-fuck
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

    print("Break from loop")
    left_motor.setVelocity(0.0)
    print("In line")
    right_motor.setVeloicy(0.0)


# Main
if __name__ == "__main__":
    my_robot = Robot()
    print("Starting Robot")
    exec_robot(my_robot)
