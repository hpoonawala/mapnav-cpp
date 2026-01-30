#/*
# * Copyright (C) 2014  RoboPeak
# * Copyright (C) 2014 - 2018 Shanghai Slamtec Co., Ltd.
# *
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * (at your option) any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *
# */
#
EIGEN := -I$(CURDIR)/../eigen-3.4.0 
C_INCLUDES += -I$(CURDIR)/../rplidar_sdk/sdk/include -I$(CURDIR)/../rplidar_sdk/sdk/src -I../old-mapnav-cpp/asio-1.36.0/include/ $(EIGEN) 

EXTRA_OBJ := scan_match_11.o pose.o OccupancyGrid.o slam_posegraph.o mapper.o TelemetryServer.o
LD_LIBS += -lstdc++ -lpthread -lsl_lidar_sdk -lm
LDFLAGS = -L../rplidar_sdk/output/Linux/Release/

all: scan_match_11.o pose.o OccupancyGrid.o slam_posegraph.o TelemetryServer.o lidar.exe lidar_react.exe

lidar.exe : lidar.cpp $(EXTRA_OBJ) DDRCappController.o SerialWriter.o TelemetryServer.o
	g++ -std=c++11 $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp mapper.cpp DDRCappController.cpp SerialWriter.cpp TelemetryServer.cpp $(LD_LIBS) -o $@

lidar_react.exe : lidar_react.cpp $(EXTRA_OBJ) DDRCappController.o SerialWriter.o
	g++ -std=c++11 $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp mapper.cpp DDRCappController.cpp SerialWriter.cpp  $(LD_LIBS) -o $@

main.exe : main.cpp $(EXTRA_OBJ)
	g++ -std=c++11 $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp $(LD_LIBS) -o $@

main_load.exe : main_load.cpp $(EXTRA_OBJ) mapper.o
	g++ -std=c++11 $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp mapper.cpp $(LD_LIBS) -o $@

scan_match_11.o : scan_match_11.cpp
	g++ --std=c++11 -c $(EIGEN) $< -o $@

mapper.o : mapper.cpp scan_match_11.o pose.o OccupancyGrid.o slam_posegraph.o
	g++ -c -std=c++11 $(EIGEN) $< -o $@

pose.o : pose.cpp
	g++ -c --std=c++11 pose.cpp -o pose.o

DDRCappController.o : DDRCappController.cpp pose.o
	g++ -std=c++11 -c $(EIGEN) $< -o $@

OccupancyGrid.o : OccupancyGrid.cpp
	g++ -std=c++11 -c $(EIGEN) $< -o $@

slam_posegraph.o : slam_posegraph.cpp scan_match_11.o pose.o
	g++ -std=c++11 -c $(EIGEN) $< -o $@

SerialWriter.o : SerialWriter.cpp
	g++ -c --std=c++11 -I../old-mapnav-cpp/asio-1.36.0/include/ $< -o $@ -lpthread

TelemetryServer.o : TelemetryServer.cpp
	g++ -c --std=c++11 -I../old-mapnav-cpp/asio-1.36.0/include/ $(EIGEN) $< -o $@ -lpthread
