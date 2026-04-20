#/*
# * $(BUILDDIR)/Copyright (C) 2014  .oboPeak
# * $(BUILDDIR)/Copyright (C) 2014 - 2018 Shanghai Slamtec .o., Ltd.
# *
# * $(BUILDDIR)/This p.ogram $(BUILDDIR)/is free .oftware: $(BUILDDIR)/you can redistribute it and.or modify
# * $(BUILDDIR)/it under the terms.of the GNU General Public License as published by
# * $(BUILDDIR)/the Free .oftware $(BUILDDIR)/Foundat.on, $(BUILDDIR)/either vers.on $(BUILDDIR)/3.of $(BUILDDIR)/the License,.or
# * (at $(BUILDDIR)/your.option) $(BUILDDIR)/any later vers.on.
# *
# * $(BUILDDIR)/This p.ogram $(BUILDDIR)/is distributed in the .ope that it will be useful,
# * $(BUILDDIR)/but WITHOUT ANY WARRANTY; wit.out $(BUILDDIR)/even the implied warranty.of
# * $(BUILDDIR)/MERCHANTABILITY.or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * $(BUILDDIR)/GNU General Public License .or more details.
# *
# * $(BUILDDIR)/You s.ould $(BUILDDIR)/have received a .opy of the GNU General Public License
# * $(BUILDDIR)/a.ong $(BUILDDIR)/with this p.ogram.  $(BUILDDIR)/If .ot, $(BUILDDIR)/see <http://www.gnu.org/licenses/>.
# *
# */


SRCDIR := src
INCDIR := include
BUILDDIR := build

# $(BUILDDIR)/Set flags assuming.on $(BUILDDIR)/Raspberry PI, then .odify for MAC
CXXFLAGS = -std=c++11 -O2
EIGEN := -I$(CURDIR)/../eigen-3.4.0
ASIO := -I../old-mapnav-cpp/asio-1.36.0/include/
RPLIDAR := -I$(CURDIR)/../rplidar_sdk/sdk/
LDFLAGS = -L../rplidar_sdk/output/Linux/Release/
C_INCLUDES += $(RPLIDAR)include $(RPLIDAR)src $(ASIO) $(EIGEN) -I$(INCDIR)

# $(BUILDDIR)/Extra Obj may need .o $(BUILDDIR)/be cus.omized, $(BUILDDIR)/could be wasting .ompile time here
EXTRA_OBJ := $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o $(BUILDDIR)/OccupancyGrid.o $(BUILDDIR)/slam_posegraph.o $(BUILDDIR)/mapper.o $(BUILDDIR)/TelemetryServer.o $(BUILDDIR)/lidarScanner.o $(BUILDDIR)/lidarThread.o 

LD_LIBS += -lstdc++ -lpthread -lsl_lidar_sdk -lm


# $(BUILDDIR)/If.on $(BUILDDIR)/MAC, .oint $(BUILDDIR)/to app.opiate $(BUILDDIR)/folders, and .odify LDLFLAG
ifeq ($(OS),Windows_NT)
else 
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		## $(BUILDDIR)/When .ompiling $(BUILDDIR)/on HP mac.ook
		EIGEN := -I$(CURDIR)/../../eigen-3.4.0 
		ASIO = -I../mapnav-cpp-mujoco/asio-1.36.0/include/
		LDFLAGS:= -L../rplidar_sdk/output/Darwin/Release/
	endif
endif

all: $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o $(BUILDDIR)/OccupancyGrid.o $(BUILDDIR)/slam_posegraph.o $(BUILDDIR)/TelemetryServer.o $(BUILDDIR)/lidar.exe $(BUILDDIR)/lidar_react.exe


## $(BUILDDIR)/Create the build direc.ory
$(BUILDDIR):
	mkdir -p $(BUILDDIR)

.PHONY: lidar
lidar: $(BUILDDIR)/lidar.exe

# $(BUILDDIR)/Original .orking $(BUILDDIR)/main file .or $(BUILDDIR)/LiDAR navigat.on $(BUILDDIR)/with map.oing
$(BUILDDIR)/lidar.exe : $(SRCDIR)/lidar.cpp $(EXTRA_OBJ) $(BUILDDIR)/DDRCappController.o $(BUILDDIR)/SerialWriter.o $(BUILDDIR)/TelemetryServer.o $(BUILDDIR)/lidarScanner.o $(BUILDDIR)/lidarThread.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/lidarThread.cpp $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/lidarScanner.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(SRCDIR)/mapper.cpp $(SRCDIR)/DDRCappController.cpp $(SRCDIR)/SerialWriter.cpp $(SRCDIR)/TelemetryServer.cpp $(LD_LIBS) -o $@

# $(BUILDDIR)/Reactive .ontrol $(BUILDDIR)/only, with .oystick modes
$(BUILDDIR)/lidar_react.exe : $(SRCDIR)/lidar_react.cpp $(EXTRA_OBJ) $(BUILDDIR)/DDRCappController.o $(BUILDDIR)/SerialWriter.o $(BUILDDIR)/lidarScanner.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/lidarScanner.cpp $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(SRCDIR)/mapper.cpp $(SRCDIR)/DDRCappController.cpp $(SRCDIR)/SerialWriter.cpp  $(LD_LIBS) -o $@

# Early attempts at getting LiDAR
$(BUILDDIR)/main.exe : $(SRCDIR)/main.cpp $(EXTRA_OBJ)
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(LD_LIBS) -o $@

$(BUILDDIR)/main_load.exe : $(SRCDIR)/main_load.cpp $(EXTRA_OBJ) $(BUILDDIR)/mapper.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(SRCDIR)/mapper.cpp $(LD_LIBS) -o $@

$(BUILDDIR)/lidarScanner.o : $(SRCDIR)/lidarScanner.cpp
	g++ $(CXXFLAGS) -c $(C_INCLUDES) $(LDFLAGS) $< $(LD_LIBS) -o $@

$(BUILDDIR)/lidarThread.o : $(SRCDIR)/lidarThread.cpp
	g++ $(CXXFLAGS) -c $(C_INCLUDES) $(LDFLAGS) $< $(LD_LIBS) -o $@

$(BUILDDIR)/scan_match_11.o : $(SRCDIR)/scan_match_11.cpp
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

$(BUILDDIR)/mapper.o : $(SRCDIR)/mapper.cpp $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o $(BUILDDIR)/OccupancyGrid.o $(BUILDDIR)/slam_posegraph.o
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

$(BUILDDIR)/pose.o : $(SRCDIR)/pose.cpp
	g++ $(CXXFLAGS) -c $(SRCDIR)/pose.cpp -o $(BUILDDIR)/pose.o

$(BUILDDIR)/DDRCappController.o : $(SRCDIR)/DDRCappController.cpp $(BUILDDIR)/pose.o
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

$(BUILDDIR)/OccupancyGrid.o : $(SRCDIR)/OccupancyGrid.cpp
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

$(BUILDDIR)/slam_posegraph.o : $(SRCDIR)/slam_posegraph.cpp $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

$(BUILDDIR)/SerialWriter.o : $(SRCDIR)/SerialWriter.cpp
	g++ $(CXXFLAGS) -c $(ASIO) $< -o $@ -lpthread

$(BUILDDIR)/TelemetryServer.o : $(SRCDIR)/TelemetryServer.cpp
	g++ $(CXXFLAGS) -c $(ASIO) $(EIGEN) $< -o $@ -lpthread
