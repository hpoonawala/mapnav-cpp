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

# Set flags assuming on Raspberry PI, then modify for MAC
CXXFLAGS = -std=c++11 -O2
EIGEN := -I$(CURDIR)/../eigen-3.4.0
ASIO := -I../old-mapnav-cpp/asio-1.36.0/include/
RPLIDAR := -I$(CURDIR)/../rplidar_sdk/sdk/
LDFLAGS = -L../rplidar_sdk/output/Linux/Release/
C_INCLUDES += $(RPLIDAR)include $(RPLIDAR)src $(ASIO) $(EIGEN)

# Extra Obj may need to be customized, could be wasting compile time here
EXTRA_OBJ := scan_match_11.o pose.o OccupancyGrid.o slam_posegraph.o mapper.o TelemetryServer.o lidarScanner.o 
LD_LIBS += -lstdc++ -lpthread -lsl_lidar_sdk -lm

# If on MAC, point to appropiate folders, and modify LDLFLAG
ifeq ($(OS),Windows_NT)
else 
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		## When Compiling on HP macbook
		EIGEN := -I$(CURDIR)/../../eigen-3.4.0 
		ASIO = -I../mapnav-cpp-mujoco/asio-1.36.0/include/
		LDFLAGS:= -L../rplidar_sdk/output/Darwin/Release/
	endif
endif

all: scan_match_11.o pose.o OccupancyGrid.o slam_posegraph.o TelemetryServer.o lidar.exe lidar_react.exe

.PHONY: lidar
lidar: lidar.exe

# Original working main file for LiDAR navigation with mappoing
lidar.exe : lidar.cpp $(EXTRA_OBJ) DDRCappController.o SerialWriter.o TelemetryServer.o lidarScanner.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< lidarScanner.cpp OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp mapper.cpp DDRCappController.cpp SerialWriter.cpp TelemetryServer.cpp $(LD_LIBS) -o $@

# Reactive control only, with joystick modes
lidar_react.exe : lidar_react.cpp $(EXTRA_OBJ) DDRCappController.o SerialWriter.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp mapper.cpp DDRCappController.cpp SerialWriter.cpp  $(LD_LIBS) -o $@

# Early attempts at getting LiDAR
main.exe : main.cpp $(EXTRA_OBJ)
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp $(LD_LIBS) -o $@

main_load.exe : main_load.cpp $(EXTRA_OBJ) mapper.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< OccupancyGrid.cpp slam_posegraph.cpp pose.cpp scan_match_11.cpp mapper.cpp $(LD_LIBS) -o $@

lidarScanner.o : lidarScanner.cpp
	g++ $(CXXFLAGS) -c $(C_INCLUDES) $(LDFLAGS) $< $(LD_LIBS) -o $@

scan_match_11.o : scan_match_11.cpp
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

mapper.o : mapper.cpp scan_match_11.o pose.o OccupancyGrid.o slam_posegraph.o
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

pose.o : pose.cpp
	g++ $(CXXFLAGS) -c pose.cpp -o pose.o

DDRCappController.o : DDRCappController.cpp pose.o
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

OccupancyGrid.o : OccupancyGrid.cpp
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

slam_posegraph.o : slam_posegraph.cpp scan_match_11.o pose.o
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

SerialWriter.o : SerialWriter.cpp
	g++ $(CXXFLAGS) -c -I../old-mapnav-cpp/asio-1.36.0/include/ $< -o $@ -lpthread

TelemetryServer.o : TelemetryServer.cpp
	g++ $(CXXFLAGS) -c -I../old-mapnav-cpp/asio-1.36.0/include/ $(EIGEN) $< -o $@ -lpthread
