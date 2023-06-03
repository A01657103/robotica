# config.bash
#
# Copyright (C) 2023 Javier de Lope
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

# Install dependencies
sudo apt update
sudo apt install python3-cbor python3-zmq python3-opencv
# Coppelia Path
export COPPELIA=$HOME/Escritorio/robotica/coppeliaSim
# Coppelia Python Dependencies Path
export PYTHONPATH=$COPPELIA/programming/zmqRemoteApi/clients/python

# Scripts Path
export SCRIPTS=$HOME/Escritorio/robotica/robotica-master/coppelia/zmqRemoteApi/python