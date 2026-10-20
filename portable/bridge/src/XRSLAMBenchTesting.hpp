#ifndef XRSLAM_VIO_BENCH_TESTING_HPP
#define XRSLAM_VIO_BENCH_TESTING_HPP

#include "XRSLAMBench.h"

#include "Frozen/XRSLAM.h"
#include <vector>

struct xrslam_bench_upstream_api {
  int (*create)(const char *, const char *, const char *, const char *,
                void **);
  void (*push_sensor_data)(XRSLAMSensorType, void *);
  void (*run_one_frame)(void);
  void (*get_result)(XRSLAMResultType, void *);
  void (*destroy)(void);
};

xrslam_bench_status_t xrslam_bench_create_with_api_for_testing(
    const xrslam_bench_create_options_t *options,
    const xrslam_bench_upstream_api *api, xrslam_bench_t **out_bench);

#endif
