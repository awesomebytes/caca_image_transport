#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <caca.h>

class Display
{
public:
  Display() : _previous_buffer_length(0)
  {
    cv_ = caca_create_canvas(0, 0);
    if(!cv_)
    {
        ROS_ERROR("Unable to initialise libcaca");
        ros::shutdown();
    }
    display_ = caca_create_display_with_driver(cv_, "slang");

    //cv_.setColorANSI(CACA_DEFAULT, CACA_TRANSPARENT);
    //int bottom = 10, right = 10;
    //dit_.reset(new Dither(32, right, bottom, 4*right, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFFFFFFFF));
  }
  
  ~Display()
  {
    caca_free_canvas(cv_);
  }

  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    // Figure out the canvas size
    unsigned int cols = tputCall(1), lines = tputCall(0), font_width = 6, font_height = 10;
    int lines_max = cols * msg->height * font_width / msg->width / font_height;
    if (lines_max <= lines)
      lines = lines_max;
    else
      cols = lines * msg->width * font_height / msg->height / font_width;

    // Create the canvas
    caca_set_canvas_size(cv_, cols, lines);
    caca_set_color_ansi(cv_, CACA_DEFAULT, CACA_TRANSPARENT);
    caca_clear_canvas(cv_);

    // Convert the image to RGB or Grayscale
    sensor_msgs::ImagePtr img;
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
      img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->toImageMsg();
    else
      img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->toImageMsg();

    unsigned int depth = sensor_msgs::image_encodings::numChannels(img->encoding);
    int bpp = sensor_msgs::image_encodings::bitDepth(img->encoding) * depth, rmask, gmask, bmask, amask = 0;
    rmask = 0x00ff0000;
    gmask = 0x0000ff00;
    bmask = 0x000000ff;
    if (sensor_msgs::image_encodings::hasAlpha(img->encoding))
      amask = 0xff000000;

    caca_dither *dither = caca_create_dither(bpp, img->width, img->height, img->step,
                                     rmask, gmask, bmask, amask);
    caca_set_dither_algorithm(dither, "fstein");
    caca_dither_bitmap(cv_, 0, 0, cols, lines, dither, reinterpret_cast<const void*>(&img->data[0]));

    size_t len;
    char format[] = "utf8";
    void *export_buffer = caca_export_canvas_to_memory(cv_, format, &len);
    if(!export_buffer)
    {
      ROS_ERROR("Can't export to format '%s'", format);
    }
    else
    {
      //std::cout << std::string(_previous_buffer_length, '\b') << std::cout << std::string(reinterpret_cast<char *>(export_buffer), len) << std::endl;
        caca_refresh_display(display_);
      _previous_buffer_length = len;
      free(export_buffer);
    }
  }

private:
  int tputCall(int line_col)
  {
    FILE* pipe;
    if (line_col == 0)
      pipe = popen("tput lines", "r");
    else
      pipe = popen("tput cols", "r");
    if (!pipe)
      return 0;
    char buffer[128];
    std::string result = "";
    int res;
    while (!feof(pipe)) {
      if (fgets(buffer, 128, pipe) != NULL)
        res = atoi(buffer);
    }
    pclose(pipe);
    return res;
  }

  caca_canvas_t *cv_;
  int _previous_buffer_length;
  caca_display_t *display_;
};

int
main(int argc, char** argv)
{
  char const * const *list;
  list = caca_get_display_driver_list();
  for(int i = 0; list[i]; i += 2)
  {
    std::cout << list[i] << std::endl;
  }

  ros::init(argc, argv, "image_view_terminal");
  ros::NodeHandle nh;

  // Make our node available to sigintHandler
  ros::NodeHandle nh_;
  std::string image_topic;
  nh_.param("image", image_topic, std::string("/camera/rgb/image_color"));

  // initialize 
  Display display;
  ROS_INFO("Caca initialized");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(image_topic, 1, boost::bind(&Display::callback, &display, _1));

  ros::spin();
}
