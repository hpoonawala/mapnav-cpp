#ifndef SLAMTHREAD_H
#define SLAMTHREAD_H
#include <stdlib.h>
#include <vector>
#include <future>
#include <mutex>
#include "../include/scan_match_11.h"
#include "../include/pose.h"
#include "OccupancyGrid.h"
#include "slam_posegraph.h"
#include "frameHistory.h"
 

class SlamThread {
  public:
      SlamThread(NDTScanMatcher&, Pose2D, double, int, OccupancyGrid);
	  ~SlamThread();
      // Called from Thread 2. Takes snapshot, launches if idle. Returns false if busy.
      bool tryLaunch(FrameHistory&,const Pose2D&);
      // Called from Thread 2. Non-blocking poll. Returns true if results are fresh.
      bool tryCollect(FrameHistory&, OccupancyGrid&, Pose2D&, vector<pair<double,double>>&);
      void wait(); // for shutdown — blocks until any in-flight job finishes
  private:
      PoseGraph posegraph_;
      int ind_interval_;
      std::future<void> future_;
      // Stored by the async job, read by tryCollect
      std::vector<Pose2D> corrected_poses_;
      std::vector<int>  nodes_;
      std::vector<Frame> snapshot_;  // held so tryCollect has the scans
	  OccupancyGrid new_grid_;
	  std::vector<std::pair<double,double>> new_path_;
};
#endif
