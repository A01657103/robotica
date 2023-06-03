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
s
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy as np
import cv2
import robotica


def detect_ball(robot):
    # Get the image from the camera
    image = robot.get_image()
    # Mask the image
    image, mask = mask_image(image)
    # Find contours in the mask
    contours = get_contours(mask)
    # Center coordinates
    cx, cy = 0, 0
    # Default moment dictionary with "m00" set to prevent division by zero
    M = {"m00": 1}
    # Draw the contours on the image
    for contour in contours:
        # Calculate image moments of the detected contour
        M = cv2.moments(contour)
        if M["m00"] != 0:
            # Calculate x,y coordinate of center
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # Draw the center of the object in green
            cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1)
            # Draw the contour on the image
            cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

    return image, cx, M["m00"]


def get_contours(mask):
    # Find contours in the mask
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # Sort the contours by area in descending order. Keep only the largest.
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
    # Return the contours
    return contours


def get_speed(cx, area):
    # Default speed
    default_speed = 4
    # Max Area
    max_area = 40000
    # Set the speed of the robot
    speed = default_speed
    # If the area is greater than the max area, slow down the robot
    print(area)
    if area > max_area:
        speed = default_speed - 1
    # Compute the error between the desired cx (center) and the current cx
    error = (128 - cx) / 128
    # Adjustment
    adjustment = error * 2
    # Adjust the left and right speed based on the error
    rspeed = speed + adjustment
    lspeed = speed - adjustment
    # Return the left and right speed
    return lspeed, rspeed


def mask_image(image):
    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Define the color range for the red color
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    # Create a mask for the red color
    mask = cv2.inRange(hsv_image, lower_red, upper_red)
    # Bitwise-AND mask and original image
    image = cv2.bitwise_and(image, image, mask=mask)
    return image, mask


def show_image(img):
    cv2.imshow('opencv', img)
    cv2.waitKey(1)


def start_simulation():
    # Create a CoppeliaSim instance
    coppelia = robotica.Coppelia()
    # Create a Pioneer P3DX instance
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX', True)
    # Start the simulation
    coppelia.start_simulation()
    # Return the CoppeliaSim and Pioneer P3DX instances
    return coppelia, robot


def main(args=None):
    # Start the simulation
    coppelia, robot = start_simulation()
    # Loop until the CoppeliaSim instance stops running
    while coppelia.is_running():
        # Detect the ball, mask the image, get the center coordinates and the
        # area.
        img, cx, area = detect_ball(robot)
        # Show the masked image of the ball
        show_image(img)
        # Calculate the left and right speed
        lspeed, rspeed = get_speed(cx, area)
        # Set the speed of the robot
        robot.set_speed(lspeed, rspeed)
    # Stop the simulation
    coppelia.stop_simulation()
    # Close all windows
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
