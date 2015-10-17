#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

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
    unsigned int cols = tputCall(1), lines = tputCall(0), font_width = 6, font_height = 10;
    int lines_max = cols * msg->height * font_width / msg->width / font_height;
    if (lines_max <= lines)
      lines = lines_max;
    else
      cols = lines * msg->width * font_height / msg->height / font_width;

    caca_set_canvas_size(cv_, cols, lines);
    caca_set_color_ansi(cv_, CACA_DEFAULT, CACA_TRANSPARENT);
    caca_clear_canvas(cv_);

    unsigned int depth = sensor_msgs::image_encodings::numChannels(msg->encoding), bpp = sensor_msgs::image_encodings::bitDepth(msg->encoding), rmask, gmask, bmask, amask = 0;
    bmask = 0x00ff0000;
    gmask = 0x0000ff00;
    rmask = 0x000000ff;
    if (sensor_msgs::image_encodings::hasAlpha(msg->encoding))
      amask = 0xff000000;

    caca_dither *dither = caca_create_dither(bpp, msg->width, msg->height, depth * msg->width,
                                     rmask, gmask, bmask, amask);
    caca_set_dither_algorithm(dither, "fstein");
    caca_dither_bitmap(cv_, 0, 0, cols, lines, dither, reinterpret_cast<const void*>(&msg->data[0]));

    size_t len;
    char format[] = "utf8";
    void *export_buffer = caca_export_canvas_to_memory(cv_, format, &len);
    if(!export_buffer)
    {
        ROS_ERROR("Can't export to format '%s'", format);
    }
    else
    {
        ROS_INFO_STREAM(_previous_buffer_length);
      if (_previous_buffer_length > 0)
        std::cout << std::string(_previous_buffer_length, '\b') << std::endl;
      _previous_buffer_length = len;
      fwrite(export_buffer, len, 1, stdout);
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
};

int
main(int argc, char** argv)
{
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
