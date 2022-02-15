from controller import Robot, Motor, Compass, DistanceSensor, TouchSensor

# Global Time-step Constant
TIMESTEP = 64


# Controller function for robot
def exec_robot(robot):
    # this is specified in the documentation
    max_speed = 6.28

    # get the motor, enable, and set motor devices
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

    # main loop
    while robot.step(TIMESTEP) != -1:
        for i in range(8):
            print("Index: {}, Value: {}".format(i, sensors[i].getValue()))

        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)


# Main
if __name__ == "__main__":
    my_robot = Robot()
    print("Starting Robot")
    exec_robot(my_robot)
