#pragma once
#include <vector> // Exact official XRSLAM.h requires the includer to provide this.
#include "XRSLAMBenchTesting.hpp"
#include "replay_loop.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <dataset_reader.h>
struct OriginalIOState {
 std::atomic<bool> active{false},cancel{false},natural_eof{false},incomplete{true},failed{false};
 std::atomic<uint64_t> camera{0},gyro{0},accel{0},frames{0},poses{0},again{0};
 // Separate narrow cancellation lock: never held while observing core locks.
 std::mutex cancel_mutex;
 std::condition_variable cancel_cv;
 void request_cancel() {
  {std::lock_guard<std::mutex> lock(cancel_mutex);cancel.store(true);}
  cancel_cv.notify_all();
 }
 void wait_terminal_poll() {
  std::unique_lock<std::mutex> lock(cancel_mutex);
  cancel_cv.wait_for(lock,std::chrono::milliseconds(100),[this]{return cancel.load();});
 }
 void reset() { cancel=false;natural_eof=false;incomplete=true;failed=false;camera=0;gyro=0;accel=0;frames=0;poses=0;again=0; }
 void snapshot(xrslam_bench_original_io_snapshot_t& s) const {
  s={sizeof(s),active.load(),cancel.load(),natural_eof.load(),incomplete.load(),failed.load(),camera.load(),gyro.load(),accel.load(),frames.load(),poses.load(),again.load()};
 }
};
struct OriginalIOCancelled {};
// This gate adds only cancellation. END is never fabricated. During cancellation
// the unmodified reader is drained separately, with no estimator/backend calls.
struct OriginalIOReaderGate {
 DatasetReader& reader;OriginalIOState& status;
 enum { AGAIN=DatasetReader::AGAIN,CAMERA=DatasetReader::CAMERA,GYROSCOPE=DatasetReader::GYROSCOPE,ACCELEROMETER=DatasetReader::ACCELEROMETER,END=DatasetReader::END };
 DatasetReader::NextDataType next() {
  if(status.cancel.load()) throw OriginalIOCancelled{};
  auto t=reader.next();if(t==DatasetReader::AGAIN)++status.again;return t;
 }
 auto read_image(){return reader.read_image();}
 auto read_gyroscope(){return reader.read_gyroscope();}
 auto read_accelerometer(){return reader.read_accelerometer();}
};
// Called after a failed/cancelled loop, before the Async reader destructor joins.
void drain_original_reader(DatasetReader& reader);
xrslam_bench_status_t replay_original_io(const xrslam_bench_upstream_api&,void*,const char*,const char*,OriginalIOState&);
// Bounded fixture seam: shares the exact gate/loop/backend/writer implementation.
xrslam_bench_status_t replay_original_reader_for_testing(const xrslam_bench_upstream_api&,DatasetReader&,const char*,OriginalIOState&);
