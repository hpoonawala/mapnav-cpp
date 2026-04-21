#ifndef SLAMTHREAD_H
#define SLAMTHREAD_H
#include <stdlib.h>
#include <vector>
#include <future>
#include "../include/scan_match_11.h"
#include "../include/pose.h"
#include "OccupancyGrid.h"
#include "slam_posegraph.h"
#include "frameHistory.h"
 
class SlamThread {
  public:
      SlamThread(NDTScanMatcher&, Pose, double, int);
	  ~SlamThread();
      // Called from Thread 2. Takes snapshot, launches if idle. Returns false if busy.
      bool tryLaunch(FrameHistory& frame_history);
      // Called from Thread 2. Non-blocking poll. Returns true if results are fresh.
      bool tryCollect(FrameHistory& frame_history);
      void wait(); // for shutdown — blocks until any in-flight job finishes
  private:
      PoseGraph posegraph_;
      int ind_interval_;
      std::future<void> future_;
      // Stored by the async job, read by tryCollect
      std::vector<Pose> corrected_poses_;
      std::vector<int>  nodes_;
      std::vector<Frame> snapshot_;  // held so tryCollect has the scans
  };
#endif
