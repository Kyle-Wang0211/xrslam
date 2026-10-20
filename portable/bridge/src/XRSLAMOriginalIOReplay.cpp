#include "XRSLAMOriginalIOReplay.h"
#include <euroc_dataset_reader.h>
#include <trajectory_writer.h>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <stdexcept>
namespace {
struct Backend {
 static constexpr XRSLAMState tracking_success=XRSLAM_STATE_TRACKING_SUCCESS;
 const xrslam_bench_upstream_api& api;OriginalIOState& s;TumTrajectoryWriter writer;
 Backend(const xrslam_bench_upstream_api& a,OriginalIOState& state,const char* path):api(a),s(state),writer(path){}
 void check(){if(s.cancel.load())throw OriginalIOCancelled{};}
 void gyroscope(XRSLAMGyroscope gyro){check();api.push_sensor_data(XRSLAM_SENSOR_GYROSCOPE,&gyro);++s.gyro;}
 void acceleration(XRSLAMAcceleration acc){check();api.push_sensor_data(XRSLAM_SENSOR_ACCELERATION,&acc);++s.accel;}
 void camera(double time,const cv::Mat& img){
  check();if(img.empty())throw std::runtime_error("Empty original image");
  XRSLAMImage image;image.camera_id=0;image.timeStamp=time;image.ext=nullptr;
  image.data=img.data;image.channel=img.channels();image.stride=img.step[0];
  api.push_sensor_data(XRSLAM_SENSOR_CAMERA,&image);++s.camera;
 }
 void run_one_frame(){check();api.run_one_frame();++s.frames;}
 XRSLAMState state(){check();XRSLAMState result;api.get_result(XRSLAM_RESULT_STATE,&result);return result;}
 XRSLAMPose body_pose(){check();XRSLAMPose pose;api.get_result(XRSLAM_RESULT_BODY_POSE,&pose);return pose;}
 void write_pose(double time,const XRSLAMPose& pose){writer.write_pose(time,pose);++s.poses;}
};
void reserve_output(const char* output){
 int fd=open(output,O_WRONLY|O_CREAT|O_EXCL,0600);
 if(fd<0)throw std::runtime_error("Cannot reserve fresh output");
 if(close(fd))throw std::runtime_error("Cannot close fresh output reservation");
}
}
void drain_original_reader(DatasetReader& reader){
 for(auto t=reader.next();t!=DatasetReader::END;t=reader.next())switch(t){
 case DatasetReader::CAMERA:(void)reader.read_image();break;
 case DatasetReader::GYROSCOPE:(void)reader.read_gyroscope();break;
 case DatasetReader::ACCELEROMETER:(void)reader.read_accelerometer();break;
 case DatasetReader::AGAIN:break;
 default:throw std::runtime_error("Unexpected drain event");
 }
}
xrslam_bench_status_t replay_original_reader_for_testing(const xrslam_bench_upstream_api& api,DatasetReader& reader,const char* output,OriginalIOState& s){
 try{
  reserve_output(output);
  Backend backend(api,s,output);OriginalIOReaderGate gate{reader,s};
  replay_stream(gate,backend);
  // Reader EOF closes the official writer on return. Completion remains
  // provisional until the bench observes the retained core's quiescence.
  s.natural_eof=true;s.incomplete=true;
  return s.poses.load()==0?XRSLAM_BENCH_NO_OUTPUT:XRSLAM_BENCH_OK;
 }catch(const OriginalIOCancelled&){
  s.incomplete=true;
  try{drain_original_reader(reader);}catch(...){s.failed=true;return XRSLAM_BENCH_INTERNAL_ERROR;}
  return XRSLAM_BENCH_INPUT_SEALED;
 }catch(...){
  s.failed=true;s.incomplete=true;
  try{drain_original_reader(reader);}catch(...){}
  return XRSLAM_BENCH_INTERNAL_ERROR;
 }
}
xrslam_bench_status_t replay_original_io(const xrslam_bench_upstream_api& api,void* config,const char* mav0,const char* output,OriginalIOState& s){
 try{
  if(!config)throw std::runtime_error("Null core config");
  for(const char* leaf:{"/cam0/data.csv","/imu0/data.csv"})
   if(!std::filesystem::is_regular_file(std::string(mav0)+leaf))throw std::runtime_error("Missing CSV");
  // The root freezes all payload hashes before runtime. Check path availability
  // before entering the original worker, whose exceptions are not transported.
  CameraCsv camera;camera.load(std::string(mav0)+"/cam0/data.csv");
  for(const auto& item:camera.items)
   if(!std::filesystem::is_regular_file(std::string(mav0)+"/cam0/data/"+item.filename))throw std::runtime_error("Missing PNG");
  auto reader=DatasetReader::create_reader(std::string("euroc://")+mav0,
    new xrslam::extra::YamlConfig(*static_cast<xrslam::extra::YamlConfig*>(config)));
  if(!reader)throw std::runtime_error("Null reader");
  return replay_original_reader_for_testing(api,*reader,output,s);
 }catch(...){s.failed=true;s.incomplete=true;return XRSLAM_BENCH_INTERNAL_ERROR;}
}
