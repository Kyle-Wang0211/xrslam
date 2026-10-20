#pragma once
#include <cstddef>
#include <stdexcept>
struct ReplayCounts { std::size_t camera=0, gyro=0, accel=0, frames=0, poses=0; };
template<class Reader,class Backend>
ReplayCounts replay_stream(Reader& reader,Backend& backend) {
 ReplayCounts count;
 bool has_gyroscope=false,has_accelerometer=false;
 for(auto type=reader.next();type!=Reader::END;type=reader.next()) {
  switch(type) {
   case Reader::AGAIN: break;
   case Reader::GYROSCOPE: {
    auto [time,gyro]=reader.read_gyroscope();
    (void)time; backend.gyroscope(gyro); has_gyroscope=true; ++count.gyro;
   } break;
   case Reader::ACCELEROMETER: {
    auto [time,acc]=reader.read_accelerometer();
    (void)time; backend.acceleration(acc); has_accelerometer=true; ++count.accel;
   } break;
   case Reader::CAMERA: {
    auto [time,image]=reader.read_image(); backend.camera(time,image); ++count.camera;
    if(has_gyroscope && has_accelerometer) {
     backend.run_one_frame(); ++count.frames;
     if(backend.state()==Backend::tracking_success) {
      auto pose=backend.body_pose();
      if(pose.timestamp>0) { backend.write_pose(pose.timestamp,pose); ++count.poses; }
     }
    }
   } break;
   default: throw std::runtime_error("Unexpected reader event");
  }
 }
 return count;
}


