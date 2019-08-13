#include <SLAMBenchAPI.h>
#include <io/IdentityFrame.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/FrameBuffer.h>
#include <random>

double probability = 0;
double default_probability = 0.1;
static slambench::io::CameraSensor *grey_sensor;

bool sb_new_filter_configuration (SLAMBenchFilterLibraryHelper *filter_settings)
{
  // initialise filter with probability parameter
  filter_settings->addParameter(TypedParameter<double>("fth",
                                                       "skip-probability",
                                                       "Numerical value to specify probability of dropping frame",
                                                       &probability,
                                                       &default_probability));
  return true;
}

bool sb_init_filter (SLAMBenchFilterLibraryHelper *filter_settings) {
  // Initialise sensors
  slambench::io::CameraSensorFinder sensor_finder;
  grey_sensor = sensor_finder.FindOne(filter_settings->get_sensors(),
                                      {{"camera_type", "grey"}});
  assert(grey_sensor);
  
  return true;
}

slambench::io::SLAMFrame * sb_process_filter (SLAMBenchFilterLibraryHelper *,
                                              slambench::io::SLAMFrame *frame) {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  slambench::io::SLAMFrame *new_frame = nullptr;
  int upper_range = 1 / probability;
  std::uniform_int_distribution<> dis(1, upper_range);
  
  if (dis(gen) == 1) {
    std::cout << "** Skip one frame." << std::endl; // skip frame
  } else {
    new_frame = new IdentityFrame(frame); // assigns new_frame to a copy of curent_frame
  }
  
  return new_frame;
}
