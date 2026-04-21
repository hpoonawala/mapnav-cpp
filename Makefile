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
EXTRA_OBJ := $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o $(BUILDDIR)/OccupancyGrid.o $(BUILDDIR)/slam_posegraph.o $(BUILDDIR)/mapper.o $(BUILDDIR)/TelemetryServer.o $(BUILDDIR)/lidarScanner.o $(BUILDDIR)/lidarThread.o $(BUILDDIR)/MotorController.o

LD_LIBS += -lstdc++ -lpthread -lsl_lidar_sdk -lm


# $(BUILDDIR)/If.on $(BUILDDIR)/MAC, .oint $(BUILDDIR)/to app.opiate $(BUILDDIR)/folders, and .odify LDLFLAG
ifeq ($(OS),Windows_NT)
else 
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		## When compiling on HP macbook
		EIGEN := -I$(CURDIR)/../../eigen-3.4.0 
		ASIO = -I../mapnav-cpp-mujoco/asio-1.36.0/include/
		LDFLAGS:= -L../rplidar_sdk/output/Darwin/Release/
	endif
endif

all: $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o $(BUILDDIR)/OccupancyGrid.o $(BUILDDIR)/slam_posegraph.o $(BUILDDIR)/TelemetryServer.o $(BUILDDIR)/lidar.exe $(BUILDDIR)/lidar_react.exe


.PHONY: lidar
lidar: $(BUILDDIR) $(BUILDDIR)/lidar.exe 

## $(BUILDDIR)/Create the build direc.ory
$(BUILDDIR):
	mkdir -p $(BUILDDIR)

# Original working main file for LiDAR navigation with mapping
$(BUILDDIR)/lidar.exe : $(SRCDIR)/lidar.cpp $(EXTRA_OBJ) $(BUILDDIR)/DDRCappController.o $(BUILDDIR)/SerialWriter.o $(BUILDDIR)/TelemetryServer.o $(BUILDDIR)/lidarScanner.o $(BUILDDIR)/lidarThread.o $(BUILDDIR)/MotorController.o $(BUILDDIR)/slamThread.o 
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $^ $(LD_LIBS) -o $@

# Reactive control only, with joystick modes
$(BUILDDIR)/lidar_react.exe : $(SRCDIR)/lidar_react.cpp $(EXTRA_OBJ) $(BUILDDIR)/DDRCappController.o $(BUILDDIR)/SerialWriter.o $(BUILDDIR)/lidarScanner.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $^  $(LD_LIBS) -o $@

# Reactive control only, with joystick modes and a watchdog timer
$(BUILDDIR)/wtchdog_lidar.exe : $(SRCDIR)/wtchdog_lidar.cpp $(EXTRA_OBJ) $(BUILDDIR)/DDRCappController.o $(BUILDDIR)/SerialWriter.o $(BUILDDIR)/lidarScanner.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/lidarScanner.cpp $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(SRCDIR)/mapper.cpp $(SRCDIR)/DDRCappController.cpp $(SRCDIR)/SerialWriter.cpp  $(LD_LIBS) -o $@

# Early attempts at getting LiDAR
$(BUILDDIR)/main.exe : $(SRCDIR)/main.cpp $(EXTRA_OBJ)
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(LD_LIBS) -o $@

$(BUILDDIR)/main_load.exe : $(SRCDIR)/main_load.cpp $(EXTRA_OBJ) $(BUILDDIR)/mapper.o
	g++ $(CXXFLAGS) $(C_INCLUDES) $(LDFLAGS) $< $(SRCDIR)/OccupancyGrid.cpp $(SRCDIR)/slam_posegraph.cpp $(SRCDIR)/pose.cpp $(SRCDIR)/scan_match_11.cpp $(SRCDIR)/mapper.cpp $(LD_LIBS) -o $@

$(BUILDDIR)/lidarScanner.o : $(SRCDIR)/lidarScanner.cpp
	g++ $(CXXFLAGS) -c $(C_INCLUDES) $< -o $@

$(BUILDDIR)/lidarThread.o : $(SRCDIR)/lidarThread.cpp
	g++ $(CXXFLAGS) -c $(C_INCLUDES) $< -o $@

$(BUILDDIR)/scan_match_11.o : $(SRCDIR)/scan_match_11.cpp
	g++ $(CXXFLAGS) -c $(EIGEN) $< -o $@

$(BUILDDIR)/mapper.o : $(SRCDIR)/mapper.cpp $(BUILDDIR)/scan_match_11.o $(BUILDDIR)/pose.o $(BUILDDIR)/OccupancyGrid.o $(BUILDDIR)/slam_posegraph.o $(BUILDDIR)/slamThread.o
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
	g++ $(CXXFLAGS) -c $(ASIO) $< -o $@ 

$(BUILDDIR)/MotorController.o : $(SRCDIR)/MotorController.cpp
	g++ $(CXXFLAGS) -c $(ASIO) $< -o $@

$(BUILDDIR)/TelemetryServer.o : $(SRCDIR)/TelemetryServer.cpp
	g++ $(CXXFLAGS) -c $(ASIO) $(EIGEN) $< -o $@ 

$(BUILDDIR)/slamThread.o : $(SRCDIR)/slamThread.cpp
	g++ $(CXXFLAGS) -c $(C_INCLUDES) $< -o $@ 

clean:
	rm -r build/
