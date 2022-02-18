from controller import Robot, TouchSensor, Device, Motor, sys


class RobotOperator:
    # constructor / enable devices
    def __init__(self):
        # constants
        self.TIMESTEP = 64
        # 6.28 is the max speed, but is too fast to properly run
        self.MAXSPEED = 6.28 / 4

        # set and enable motors and touch sensor
        self.robot = Robot()
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.left_motor.setPosition(float('inf'))

        self.right_motor = self.robot.getDevice('right wheel motor')
        self.right_motor.setPosition(float('inf'))

        self.touch = self.robot.getDevice('touch sensor')
        self.touch.enable(self.TIMESTEP)

        self.sensors = []
        for i in range(8):
            sensor_name = 'ps' + str(i)
            self.sensors.append(self.robot.getDevice(sensor_name))
            self.sensors[i].enable(self.TIMESTEP)

    def set_velocity(self, left_velocity, right_velocity):
        self.left_motor.setVelocity(left_velocity)
        self.right_motor.setVelocity(right_velocity)

    def contact(self):
        if self.touch.getValue() > 0:
            return True
        else:
            return False

    def wall_front_right(self):
        if self.sensors[0].getValue() > 250:
            return True
        else:
            return False

    def wall_right_diagonal(self):
        if self.sensors[1].getValue() > 500:
            return True
        else:
            return False

    def wall_right(self):
        if self.sensors[2].getValue() > 180:
            return True
        else:
            return False

    def wall_front_left(self):
        if self.sensors[7].getValue() > 250:
            return True
        else:
            return False

    def wall_left_diagonal(self):
        if self.sensors[6].getValue() > 500:
            return True
        else:
            return False

    def wall_left(self):
        if self.sensors[5].getValue() > 180:
            return True
        else:
            return False

    def right_hand_rule(self):
        while self.robot.step(self.TIMESTEP) != -1:
            # set initial velocity to zero
            self.set_velocity(0, 0)

            # print each sensor and its corresponding value
            for i in range(8):
                print("Sensor: {}, Value: {}".format(i, self.sensors[i].getValue()))

            if self.contact():
                print("Win")
                self.set_velocity(0, 0)
                sys.exit(0)
            elif self.wall_front_right() or self.wall_right_diagonal():
                print("Rotate Left")
                self.set_velocity(-self.MAXSPEED, self.MAXSPEED)
            else:
                if self.wall_right():
                    print("Advance")
                    self.set_velocity(self.MAXSPEED * 2, self.MAXSPEED * 2)
                else:
                    print("Turn Right")
                    self.set_velocity(self.MAXSPEED * 3, 0)

    def left_hand_rule(self):
        while self.robot.step(self.TIMESTEP) != -1:
            # set initial velocity to zero
            self.set_velocity(0, 0)

            # print each sensor and its corresponding value
            for i in range(8):
                print("Sensor: {}, Value: {}".format(i, self.sensors[i].getValue()))

            if self.contact():
                print("Win")
                self.set_velocity(0, 0)
                sys.exit(0)
            elif self.wall_front_left() or self.wall_left_diagonal():
                print("Rotate Right")
                self.set_velocity(self.MAXSPEED, -self.MAXSPEED)
            else:
                if self.wall_left():
                    print("Advance")
                    self.set_velocity(self.MAXSPEED * 2, self.MAXSPEED * 2)
                else:
                    print("Turn Left")
                    self.set_velocity(0, self.MAXSPEED * 3)


def main():
    new_robot = RobotOperator()
    # new_robot.right_hand_rule()
    new_robot.left_hand_rule()


if __name__ == '__main__':
    main()
