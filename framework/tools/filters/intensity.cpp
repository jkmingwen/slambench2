#include <SLAMBenchAPI.h>
#include <io/IdentityFrame.h>
#include <io/sensor/CameraSensorFinder.h>
#include <io/FrameBuffer.h>

double threshold = 0;
double default_threshold = 0.005;
static slambench::io::CameraSensor *grey_sensor;
bool is_first_frame_processed = false;

std::vector<float> hist_old;
float contrib; // assuming that all frames have the same resolution

void calc_histogram(slambench::io::SLAMFrame *frame, std::vector<float> &hist, float contrib)
{
  std::fill(hist.begin(), hist.end(), 1e-10);
  
  // get start and end pointers for given frame
  unsigned char *imageStart = (unsigned char *) frame->GetData(); // get pointer to first pixel value
  unsigned char *imageEnd = imageStart + frame->GetSize(); // add respective frame size
  
  // loop through histogram and add occurances of values 0-255 to respective bins
  std::for_each(imageStart, imageEnd, [&](unsigned char intensity)
                                      {
                                        if (intensity != 0)
                                          {
                                            hist[intensity - 1] += contrib;
                                          }
                                      });
  frame->FreeData();
}

float calc_kl_divergence(const std::vector<float> &hist_new,
                         const std::vector<float> &hist_old)
{
  float kl_divergence = 0;
  for (int bin = 0; bin < 255; bin++)
    {
      kl_divergence += hist_new[bin] * log(hist_new[bin] / hist_old[bin]); // they use log2 for some reason
    }
  return kl_divergence;
}

bool sb_new_filter_configuration (SLAMBenchFilterLibraryHelper *filter_settings)
{
  // initialise filter with threshold parameter
  filter_settings->addParameter(TypedParameter<double>("fth",
                                                       "skip-threshold",
                                                       "Numerical value to specify KL divergence threshold",
                                                       &threshold,
                                                       &default_threshold));
  return true;
}

bool sb_init_filter (SLAMBenchFilterLibraryHelper *filter_settings)
{
  // Initialise sensors
  slambench::io::CameraSensorFinder sensor_finder;
  grey_sensor = sensor_finder.FindOne(filter_settings->get_sensors(),
                                      {{"camera_type", "grey"}});
  assert(grey_sensor);  // make sure sensor is found
  // slambench::io::SLAMFrame *first_frame;
  // first_frame = filter_settings->get_input_interface()->GetFrames(); // need to get first frame
  // assign first_frame to first frame from grey_sensor
  /* each occurance of a pixel value will "contribute" this much to height of corresponding bin.
     note that GetSize() currently returns Width*Height*Pixelformat so it may not be the right number */

  // // Initialise histogram for first frame
  // lastIntensityDistribution = std::vector<float>(255); // might need to be 256 as values are 0-255
  // calc_histogram(first_frame, lastIntensityDistribution, contrib);
  
  return true;
}

slambench::io::SLAMFrame * sb_process_filter (SLAMBenchFilterLibraryHelper * , slambench::io::SLAMFrame * frame) {
  if (frame->FrameSensor != (slambench::io::Sensor *) grey_sensor) {
    return new IdentityFrame(frame);
  }
  float contrib = 1 / (float) (frame->GetSize()); // set contribution according to first frame resolution
  if (!hist_old.size()) {
    hist_old.resize(255);
    calc_histogram(frame, hist_old, contrib);
  }
   
  slambench::io::SLAMFrame *new_frame = nullptr;
  std::vector<float> hist_new = std::vector<float>(255);
  // calculate KL divergence from previous histogram
  calc_histogram(frame, hist_new, contrib);
  float kl_divergence = calc_kl_divergence(hist_new, hist_old);
  std::cout << "kl_divergence = " << kl_divergence << std::endl;
  if (kl_divergence < threshold) {
    std::cout << "** Skip one frame." << std::endl; // skip frame
    // note that lastIntensityDistribution has been set to new frame within calc_kl_divergence
  } else {
    hist_old = hist_new;
    new_frame = new IdentityFrame(frame); // assigns new_frame to a copy of curent_frame
  }
  
  return new_frame;
}
