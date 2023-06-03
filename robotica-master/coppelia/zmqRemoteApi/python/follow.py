'''
camera.py

Sample client for the Pioneer P3DX mobile robot that receives and
displays images from the camera.

Copyright (C) 2023 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import cv2
import robotica


def detect_red_ball(self, img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Define the color range for red in HSV
    lower_red = np.array([0, 70, 50])
    upper_red = np.array([10, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    mask = cv2.dilate(mask, None, iterations=2)

    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None
    if len(contours) > 0:
        # Find the largest contour
        c = max(contours, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        if M["m00"] > 0:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        else:
            center = None

    return center


# Define PID controller
class PIDController:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt
        self.error_sum = 0
        self.prev_error = 0

    def compute(self, error):
        self.error_sum += error
        error_diff = error - self.prev_error
        control = self.Kp*error + self.Ki*self.error_sum * \
            self.dt + self.Kd*error_diff/self.dt
        self.prev_error = error
        return control


def main(args=None):
    # Initialize CoppeliaSim connection
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX', True)
    pid = PIDController(0.1, 0.01, 0.5, 0.1)  # Tune these values as needed

    coppelia.start_simulation()

    while coppelia.is_running():
        img = robot.get_image()
        # Assume that the ball is the largest red object in the image
        ball_position = detect_red_ball(img)
        # The error is the horizontal distance of the ball from the center of the image
        error = img.shape[1]/2 - ball_position[0]
        speed = pid.compute(error)
        # Adjust the speed of the robot based on the PID controller
        robot.set_speed(speed, speed)

        # If there's an obstacle, stop and rotate the robot
        if robot.ultrasonic_sensors_detect_obstacle():
            robot.set_speed(0, 0)
            robot.rotate(90)

        cv2.imshow('opencv', img)
        cv2.waitKey(1)

    coppelia.stop_simulation()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
