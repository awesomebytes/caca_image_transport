#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <caca.h>

class Display
{
public:
  Display() : dither_(0), previous_terminal_check_(0, 0), font_width_(6), font_height_(10)
  {
    // Create the canvas on which the dithered image will be
    canvas_ = caca_create_canvas(0, 0);
    if(!canvas_)
    {
        ROS_ERROR("Unable to initialize libcaca");
        ros::shutdown();
    }
    caca_set_color_ansi(canvas_, CACA_DEFAULT, CACA_TRANSPARENT);

    // Define the display to display the canvas
    display_ = caca_create_display_with_driver(canvas_, "slang");
    // "slang" was chosen arbitrary. When calling caca_get_display_driver_list,
    // x11, gl: open up a new window
    // slang, ncurses: display in the current terminal
    // raw: displays garbage
    // null: displays nothing
  }

  ~Display()
  {
    // Delete what is currently displayed
    caca_clear_canvas(canvas_);
    caca_refresh_display(display_);

    // Free ressources
    caca_free_dither(dither_);
    caca_free_display(display_);
    caca_free_canvas(canvas_);
  }

  /* Callback for when we receive an image message
   * @param msg the image message
   */
  void callback(const sensor_msgs::ImageConstPtr& msg)
  {
    // Figure out the canvas size
    if (ros::Time::now() > previous_terminal_check_ + ros::Duration(0.5))
    {
      terminal_width_ = tputCall(false);
      terminal_height_ = tputCall(true);
      previous_terminal_check_ = ros::Time::now();
    }
    unsigned int cols = terminal_width_, lines = terminal_height_;
    int lines_max = cols * msg->height * font_width_ / (msg->width * font_height_);
    if (lines_max <= lines)
      lines = lines_max;
    else
      cols = lines * msg->width * font_height_ / (msg->height * font_width_);

    // Update the canvas
    caca_clear_canvas(canvas_);
    caca_set_canvas_size(canvas_, cols, lines);

    // Convert the image to RGB or Grayscale
    sensor_msgs::ImagePtr img;
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
      // This is an optimization in case our image is BGR (to avoid copying data)
      if (msg->encoding == sensor_msgs::image_encodings::BGR8)
      {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->toImageMsg();
      } else {
        img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8)->toImageMsg();
      }
    else
      img = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->toImageMsg();

    if ((image_property_.encoding != img->encoding) || (image_property_.width != img->width) ||
        (image_property_.height != img->height) || (image_property_.step != img->step))
      OnImagePropertyChangeCallback(msg);

    caca_dither_bitmap(canvas_, 0, 0, cols, lines, dither_, reinterpret_cast<const void*>(&img->data[0]));

    caca_refresh_display(display_);

    // Check whether we should exit
    caca_event_t ev;
    caca_get_event(display_, CACA_EVENT_ANY, &ev, 0);
    if (caca_get_event_type(&ev) == CACA_EVENT_KEY_PRESS)
      ros::shutdown();
  }

private:
  /* Callback called when the image dimensions change */
  void OnImagePropertyChangeCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    if (dither_)
      caca_free_dither(dither_);

    image_property_.encoding = msg->encoding;
    image_property_.width = msg->width;
    image_property_.height = msg->height;
    image_property_.step = msg->step;
    
    // Figure out the color masks
    int rmask, gmask, bmask, amask = 0;
    if (image_property_.encoding == sensor_msgs::image_encodings::BGR8)
    {
      rmask = 0x00ff0000;
      gmask = 0x0000ff00;
      bmask = 0x000000ff;
    } else {
      rmask = 0x000000ff;
      gmask = 0x0000ff00;
      bmask = 0x00ff0000;
    }

    if (sensor_msgs::image_encodings::hasAlpha(image_property_.encoding))
      amask = 0xff000000;

    // Create the dither
    unsigned int depth = sensor_msgs::image_encodings::numChannels(image_property_.encoding);
    int bpp = sensor_msgs::image_encodings::bitDepth(image_property_.encoding) * depth;

    dither_ = caca_create_dither(bpp, image_property_.width, image_property_.height, image_property_.step,
                                     rmask, gmask, bmask, amask);
    caca_set_dither_algorithm(dither_, "fstein");
  }

  /* Function to check the dimensions of the terminal
   * @param line_col true if we choose to compute the number of lines.
   *                 If false, it checks the number of columns.
   */
  static int tputCall(bool line_col)
  {
    FILE* pipe;
    if (line_col)
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

  // Dither to dither the image
  caca_dither *dither_;
  // Canvas where to dither the image
  caca_canvas_t *canvas_;
  // Display to display the image
  caca_display_t *display_;
  // Timer to check the terminal dimensions
  ros::Time previous_terminal_check_;
  // Terminal width
  int terminal_width_;
  // Terminal height
  int terminal_height_;
  // Font width
  int font_width_;
  // Font height
  int font_height_;
  // Image message used to store previous image properties
  sensor_msgs::Image image_property_;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "image_view_terminal");
  ros::NodeHandle nh("~");

  // Define the image topic
  std::string image_topic;
  nh.param("image", image_topic, std::string("/camera/rgb/image_color"));

  // initialize 
  Display display;

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe(image_topic, 1, boost::bind(&Display::callback, &display, _1));

  ros::spin();
}
