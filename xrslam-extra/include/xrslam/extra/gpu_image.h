// GPU front end for XRSLAM: OpenCvImage with preprocess / detect_keypoints / track_keypoints executed on the GPU
// (pw_gpu_frontend: bit-exact replicas of the OpenCV 4.0.1 CLAHE, pyramid, Scharr, goodFeaturesToTrack and LK).
// Built only with XRSLAM_GPU_FRONTEND; enabled at runtime by PW_XRSLAM_GPU_FRONTEND=1 (read once).
#ifndef XRSLAM_EXTRA_GPU_IMAGE_H
#define XRSLAM_EXTRA_GPU_IMAGE_H
#include <array>
#include <memory>
#include <vector>
#include <xrslam/extra/opencv_image.h>
namespace pw::gpufe { class FrontEnd; class Frame; }
namespace xrslam::extra {
class GpuImage : public OpenCvImage {
  public:
    GpuImage();
    ~GpuImage() override;
    void preprocess(double clipLimit, int width, int height) override;
    // Pipelining: called on the capture thread right after the gray image is set; submits the GPU pyramid build
    // asynchronously so the tracker thread's preprocess() only has to wait for (usually finished) work.
    void prefetch(double clipLimit, int width, int height);
    void detect_keypoints(std::vector<vector<2>> &keypoints, size_t max_points,
                          double keypoint_distance) const override;
    void track_keypoints(const Image *next_image,
                         const std::vector<vector<2>> &curr_keypoints,
                         std::vector<vector<2>> &next_keypoints,
                         std::vector<char> &result_status) const override;
    void release_image_buffer() override;
    static bool enabled();                      // PW_XRSLAM_GPU_FRONTEND=1 and a GPU front end could be created
    static pw::gpufe::FrontEnd *front_end();    // process-wide, lazily created; nullptr if unavailable
    static std::shared_ptr<OpenCvImage> create_image();   // GpuImage when enabled(), else OpenCvImage
    void ensure_cpu_pyramid() const;
  private:
    std::shared_ptr<pw::gpufe::Frame> frame_;
    std::vector<unsigned char> pyr_bytes_;   // GPU-built packed padded pyramid; image_pyramid[] / image are zero-copy views into it
    bool gpu_ok_ = false;
    bool audit_frame_ = false; bool prefetched_ = false;   // this frame is an on-device numerics audit frame (CPU path re-run and compared)
    mutable std::vector<std::array<float, 2>> gpu_nxt_, gpu_rev_, init_nxt_; mutable std::vector<unsigned char> gpu_st_, gpu_rst_;
};
} // namespace xrslam::extra
#endif
