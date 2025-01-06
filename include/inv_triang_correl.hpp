#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/functional.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/tuple.h>
#include <chrono>

class InverseTriangulation {
public:
    InverseTriangulation(const std::string& yaml_file);
    void read_images(const std::vector<cv::Mat>& left_imgs, const std::vector<cv::Mat>& right_imgs);
    Eigen::MatrixXf points3d(const std::vector<float>& x_lim, const std::vector<float>& y_lim, const std::vector<float>& z_lim, float xy_step = 1.0, float z_step = 1.0, bool visualize = false);
        // Create a 3D space of combination from linear arrays of X Y Z

    void plot_3d_points(const Eigen::VectorXf& x, const Eigen::VectorXf& y, const Eigen::VectorXf& z, const Eigen::VectorXf& color = Eigen::VectorXf(), const std::string& title = "Plot 3D of max correlation points") {
        // Plot 3D points as scatter points where color is based on Z value
        // This is a placeholder, actual plotting needs to be implemented using a suitable C++ library
    }

    void read_yaml_file();

    void save_points(const Eigen::MatrixXf& data, const std::string& filename, char delimiter = ',');

    thrust::device_vector<float> transform_gcs2ccs(const thrust::device_vector<float>& points_3d, const std::string& cam_name);

    thrust::device_vector<float> undistorted_points(const thrust::device_vector<float>& points, const Eigen::VectorXd& dist);

    std::pair<thrust::device_vector<float>, thrust::device_vector<float>> bi_interpolation(const thrust::device_vector<float>& images, const thrust::device_vector<float>& uv_points, int window_size = 3);

    std::tuple<thrust::device_vector<float>, thrust::device_vector<float>, thrust::device_vector<float>> spatial_correl(const thrust::device_vector<float>& uv_left, const thrust::device_vector<float>& uv_right, int window_size = 3)

    thrust::device_vector<float> correlation_process(const thrust::device_vector<float>& points_3d, int win_size = 3, float threshold = 0.8, bool save_points = true, bool visualize = false);

private:
    std::string yaml_file;
    thrust::device_vector<float> left_images;
    thrust::device_vector<float> right_images;
    std::map<std::string, std::map<std::string, Eigen::MatrixXd>> camera_params;
    int z_scan_step;
    size_t num_points;
    size_t max_gpu_usage;

};