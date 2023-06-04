'''
avoid.py

Sample client for the Pioneer P3DX mobile robot that implements a
kind of heuristic, rule-based controller for collision avoidance.

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

import robotica


def avoid(readings):
    print(readings)
    if (readings[3] < 1) or (readings[4] < 1):
        lspeed, rspeed = +1, -1
    elif readings[1] < 0.2:
        lspeed, rspeed = +0.4, +0.1
    elif readings[0] < 0.4:
        lspeed, rspeed = +0.1, +0.4
    elif readings[15] < 0.4:
        lspeed, rspeed = +0.1, +0.4
    else:
        lspeed, rspeed = +1, -1
    return lspeed, rspeed


def main(args=None):
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    coppelia.start_simulation()
    while coppelia.is_running():
        readings = robot.get_sonar()
        lspeed, rspeed = avoid(readings)
        robot.set_speed(lspeed, rspeed)
    coppelia.stop_simulation()


if __name__ == '__main__':
    main()
