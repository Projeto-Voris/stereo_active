#include "noise_image.hpp"
#include <omp.h>

std::vector<MonitorInfo> get_monitors() {
    std::vector<MonitorInfo> monitors;
    Display* display = XOpenDisplay(nullptr);

    if (!display) {
        std::cerr << "Unable to open X display" << std::endl;
        return monitors;
    }

    int screen = DefaultScreen(display);

    if (XineramaIsActive(display)) {
        int num_monitors;
        XineramaScreenInfo* screens = XineramaQueryScreens(display, &num_monitors);

        if (screens) {
            for (int i = 0; i < num_monitors; ++i) {
                monitors.push_back({
                    "Monitor_" + std::to_string(i),
                    screens[i].width,
                    screens[i].height,
                    screens[i].x_org,
                    screens[i].y_org
                });
            }
            XFree(screens);
        }
    } else {
        monitors.push_back({
            "DefaultMonitor",
            DisplayWidth(display, screen),
            DisplayHeight(display, screen),
            0,
            0
        });
    }

    XCloseDisplay(display);
    return monitors;
}



ImageDisplayNode::ImageDisplayNode() : Node("image_display_node") {
    this->declare_parameter<double>("frequency", 3.0);
    this->declare_parameter<double>("persistence", 0.5);
    this->declare_parameter<double>("lacunarity", 2.0);
    this->declare_parameter<double>("octave", 4);
    this->declare_parameter<std::string>("monitor_name", "DP-0");

    frequency_ = this->get_parameter("frequency").as_double();
    persistence_ = this->get_parameter("persistence").as_double();
    lacunarity_ = this->get_parameter("lacunarity").as_double();
    octave_ = this->get_parameter("octave").as_double();
    monitor_name_ = this->get_parameter("monitor_name").as_string();

    get_screen_resolution(monitor_name_);

    image_ = generate_noise_image(0);

    window_name_ = "Random Image";
    construct_window();

    change_image_service_ = this->create_service<std_srvs::srv::SetBool>(
        "pattern_change", std::bind(&ImageDisplayNode::change_image_cb, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10), std::bind(&ImageDisplayNode::timer_image_cb, this));
}

void ImageDisplayNode::get_screen_resolution(const std::string& monitor_name) {
    auto monitors = get_monitors();

    for (const auto& monitor : monitors) {
        if (monitor.name == monitor_name) {
            this->image_width_ = monitor.width;
            this->image_height_ = monitor.height;
            this->window_position_ = {monitor.x, monitor.y};
            RCLCPP_INFO(this->get_logger(),
                        "Monitor %s resolution: %dx%d, position: %dx%d",
                        monitor_name.c_str(),
                        monitor.width, monitor.height,
                        monitor.x, monitor.y);
            return;
        }
    }

    RCLCPP_WARN(this->get_logger(), "Monitor '%s' not found. Using default values.", monitor_name.c_str());
}

void ImageDisplayNode::construct_window() {
    cv::namedWindow(window_name_, cv::WINDOW_FULLSCREEN);
    cv::moveWindow(window_name_, window_position_.first, window_position_.second);
}

cv::Mat ImageDisplayNode::generate_noise_image(int seed) {
    cv::Mat noise_image(image_height_, image_width_, CV_8UC1);

    // Create a Perlin noise module
    noise::module::Perlin perlinModule;
    perlinModule.SetSeed(seed);
    perlinModule.SetFrequency(frequency_);
    perlinModule.SetPersistence(persistence_);
    perlinModule.SetLacunarity(lacunarity_);
    perlinModule.SetOctaveCount(octave_);

    // Generate noise values in parallel
    #pragma omp parallel for
    for (int i = 0; i < image_height_; ++i) {
        for (int j = 0; j < image_width_; ++j) {
            double x = static_cast<double>(i) / image_height_;
            double y = static_cast<double>(j) / image_width_;
            double noise_value = perlinModule.GetValue(x, y, 0.0);

            // Normalize noise_value to the range [0, 255]
            uchar noise_pixel = static_cast<uchar>((noise_value + 1.0) * 127.5);
            noise_image.at<uchar>(i, j) = noise_pixel;
        }
    }

    // Apply a binary threshold to create a black and white image
    double threshold_value = 150; // Threshold value for binarization
    cv::threshold(noise_image, noise_image, threshold_value, 255, cv::THRESH_BINARY);

    // Apply Gaussian blur to smooth the noise (optional, can be reduced or removed)
    int kernel = 21;
    cv::GaussianBlur(noise_image, noise_image, cv::Size(kernel, kernel), 0);
    cv::GaussianBlur(noise_image, noise_image, cv::Size(kernel, kernel), 0);
    cv::GaussianBlur(noise_image, noise_image, cv::Size(kernel, kernel), 0);

    RCLCPP_INFO(this->get_logger(), "Black and white noise image created with seed %d", seed);
    return noise_image;
}

int ImageDisplayNode::generate_random_seed() {
    std::random_device rd;
    return rd();
}

void ImageDisplayNode::change_image_cb(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                       const std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    int seed = generate_random_seed();
    image_ = generate_noise_image(seed);
    response->success = true;
}

void ImageDisplayNode::timer_image_cb() {
    cv::imshow(window_name_, image_);
    cv::waitKey(10);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
