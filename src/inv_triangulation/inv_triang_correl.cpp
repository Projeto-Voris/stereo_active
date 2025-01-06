#include <inv_triangulation_correl.hpp>

InverseTriangulation::InverseTriangulation(const std::string& yaml_file) {
    this->yaml_file = yaml_file;
    this->left_images = thrust::device_vector<float>();
    this->right_images = thrust::device_vector<float>();

    // Initialize all camera parameters in a single nested dictionary
    this->camera_params = {
        {"left", {Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd()}},
        {"right", {Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd(), Eigen::MatrixXd()}},
        {"stereo", {Eigen::MatrixXd(), Eigen::MatrixXd()}}
    };

    this->read_yaml_file();
    this->z_scan_step = 0;
    this->num_points = 0;
    this->max_gpu_usage = 1.0; // Set a default value for max GPU usage
}

void InverseTriangulation::read_images(const std::vector<cv::Mat>& left_imgs, const std::vector<cv::Mat>& right_imgs) {
    if (left_imgs.size() != right_imgs.size()) {
        throw std::runtime_error("Number of images do not match");
    }
    // Convert images to thrust::device_vector
    this->left_images = thrust::device_vector<float>(left_imgs.size() * left_imgs[0].total());
    this->right_images = thrust::device_vector<float>(right_imgs.size() * right_imgs[0].total());
    for (size_t i = 0; i < left_imgs.size(); ++i) {
        thrust::copy(left_imgs[i].begin<float>(), left_imgs[i].end<float>(), this->left_images.begin() + i * left_imgs[0].total());
        thrust::copy(right_imgs[i].begin<float>(), right_imgs[i].end<float>(), this->right_images.begin() + i * right_imgs[0].total());
    }
}

Eigen::MatrixXf InverseTriangulation::points3d(const std::vector<float>& x_lim, const std::vector<float>& y_lim, const std::vector<float>& z_lim, float xy_step, float z_step, bool visualize) {
    // Create a 3D space of combination from linear arrays of X Y Z
    std::vector<float> x_lin, y_lin, z_lin;
    for (float x = x_lim[0]; x < x_lim[1]; x += xy_step) x_lin.push_back(x);
    for (float y = y_lim[0]; y < y_lim[1]; y += xy_step) y_lin.push_back(y);
    for (float z = z_lim[0]; z < z_lim[1]; z += z_step) z_lin.push_back(z);

    Eigen::MatrixXf c_points(x_lin.size() * y_lin.size() * z_lin.size(), 3);
    int idx = 0;
    for (float x : x_lin) {
        for (float y : y_lin) {
            for (float z : z_lin) {
                c_points(idx, 0) = x;
                c_points(idx, 1) = y;
                c_points(idx, 2) = z;
                idx++;
            }
        }
    }

    if (visualize) {
        this->plot_3d_points(c_points.col(0), c_points.col(1), c_points.col(2));
    }

    this->num_points = c_points.rows();
    this->z_scan_step = z_lin.size();

    return c_points;
}

void InverseTriangulation::read_yaml_file() {
        // Read YAML file to extract cameras parameters
        YAML::Node params = YAML::LoadFile(this->yaml_file);

        // Parse the matrices
        this->camera_params["left"]["kk"] = Eigen::MatrixXd::Map(params["camera_matrix_left"].as<std::vector<double>>().data(), 3, 3);
        this->camera_params["left"]["kc"] = Eigen::MatrixXd::Map(params["dist_coeffs_left"].as<std::vector<double>>().data(), 5, 1);
        this->camera_params["left"]["r"] = Eigen::MatrixXd::Map(params["rot_matrix_left"].as<std::vector<double>>().data(), 3, 3);
        this->camera_params["left"]["t"] = Eigen::MatrixXd::Map(params["t_left"].as<std::vector<double>>().data(), 3, 1);

        this->camera_params["right"]["kk"] = Eigen::MatrixXd::Map(params["camera_matrix_right"].as<std::vector<double>>().data(), 3, 3);
        this->camera_params["right"]["kc"] = Eigen::MatrixXd::Map(params["dist_coeffs_right"].as<std::vector<double>>().data(), 5, 1);
        this->camera_params["right"]["r"] = Eigen::MatrixXd::Map(params["rot_matrix_right"].as<std::vector<double>>().data(), 3, 3);
        this->camera_params["right"]["t"] = Eigen::MatrixXd::Map(params["t_right"].as<std::vector<double>>().data(), 3, 1);

        this->camera_params["stereo"]["R"] = Eigen::MatrixXd::Map(params["R"].as<std::vector<double>>().data(), 3, 3);
        this->camera_params["stereo"]["T"] = Eigen::MatrixXd::Map(params["T"].as<std::vector<double>>().data(), 3, 1);
    }

void InverseTriangulation::save_points(const Eigen::MatrixXf& data, const std::string& filename, char delimiter = ',') {
        // Save a 2D Eigen matrix to a CSV file
        std::ofstream file(filename);
        if (file.is_open()) {
            for (int i = 0; i < data.rows(); ++i) {
                for (int j = 0; j < data.cols(); ++j) {
                    file << data(i, j);
                    if (j < data.cols() - 1) file << delimiter;
                }
                file << "\n";
            }
            file.close();
            std::cout << "Array saved to " << filename << std::endl;
        }
    }

    thrust::device_vector<float> InverseTriangulation::transform_gcs2ccs(const thrust::device_vector<float>& points_3d, const std::string& cam_name) {
        // Convert all inputs to thrust::device_vector for GPU computation
        thrust::device_vector<float> xyz_gcs = points_3d;
        Eigen::MatrixXd k = camera_params[cam_name]["kk"];
        Eigen::MatrixXd dist = camera_params[cam_name]["kc"];
        Eigen::MatrixXd rot = camera_params[cam_name]["r"];
        Eigen::MatrixXd tran = camera_params[cam_name]["t"];

        // Estimate the size of the input and output arrays
        size_t num_points = xyz_gcs.size() / 3;
        size_t bytes_per_float32 = 8;  // Simulate double-precision float usage

        // Estimate the memory required per point for transformation and intermediate steps
        size_t memory_per_point = (4 * 3 * bytes_per_float32) + (3 * bytes_per_float32);  // For xyz_gcs_1 and xyz_ccs
        size_t total_memory_required = num_points * memory_per_point;

        // Adjust the batch size based on memory limitations
        size_t points_per_batch;
        if (total_memory_required > max_gpu_usage * 1024 * 1024 * 1024) {
            points_per_batch = (max_gpu_usage * 1024 * 1024 * 1024 / memory_per_point) / 10;  // Reduce batch size more aggressively
        } else {
            points_per_batch = num_points;  // Process all points at once
        }

        // Initialize an empty list to store results (on the GPU)
        thrust::device_vector<float> uv_points_list(2 * num_points);

        // Process points in batches
        for (size_t i = 0; i < num_points; i += points_per_batch) {
            size_t end = std::min(i + points_per_batch, num_points);
            thrust::device_vector<float> xyz_gcs_batch(xyz_gcs.begin() + i * 3, xyz_gcs.begin() + end * 3);

            // Add one extra line of ones to the global coordinates
            thrust::device_vector<float> ones(end - i, 1.0f);
            thrust::device_vector<float> xyz_gcs_1(4 * (end - i));
            thrust::copy(xyz_gcs_batch.begin(), xyz_gcs_batch.end(), xyz_gcs_1.begin());
            thrust::copy(ones.begin(), ones.end(), xyz_gcs_1.begin() + 3 * (end - i));

            // Create the rotation and translation matrix
            Eigen::MatrixXd rt_matrix(4, 4);
            // Perform matrix multiplication on the GPU
            // This is a placeholder, actual matrix multiplication needs to be implemented using CUDA
            // For now, we will use a simple CPU-based multiplication for demonstration purposes
            for (size_t j = 0; j < end - i; ++j) {
                Eigen::Vector4f point(xyz_gcs_1[j * 4], xyz_gcs_1[j * 4 + 1], xyz_gcs_1[j * 4 + 2], xyz_gcs_1[j * 4 + 3]);
                Eigen::Vector4f transformed_point = rt_matrix * point;
            // Perform normalization on the GPU
            // This is a placeholder, actual normalization needs to be implemented using CUDA
            // For now, we will use a simple CPU-based normalization for demonstration purposes
            for (size_t j = 0; j < end - i; ++j) {
                float z = xyz_ccs[j * 4 + 2] + epsilon;
                xyz_ccs_norm[j * 3] = xyz_ccs[j * 4] / z;
                xyz_ccs_norm[j * 3 + 1] = xyz_ccs[j * 4 + 1] / z;
                xyz_ccs_norm[j * 3 + 2] = 1.0f;
            }
                xyz_ccs[j * 4 + 2] = transformed_point(2);
                xyz_ccs[j * 4 + 3] = transformed_point(3);
            }

            // Multiply the RT matrix with global points [X; Y; Z; 1]
            thrust::device_vector<float> xyz_ccs(4 * (end - i));
            // Perform matrix multiplication on the GPU
            // This is a placeholder, actual matrix multiplication needs to be implemented using CUDA

            // Normalize by dividing by Z to get normalized image coordinates
            float epsilon = 1e-10;  // Small value to prevent division by zero
            thrust::device_vector<float> xyz_ccs_norm(3 * (end - i));
            // Perform normalization on the GPU
            // This is a placeholder, actual normalization needs to be implemented using CUDA

            // Apply distortion using the GPU
            thrust::device_vector<float> xyz_ccs_norm_dist = undistorted_points(xyz_ccs_norm, dist);

            // Compute image points using the intrinsic matrix K
            thrust::device_vector<float> uv_points_batch(2 * (end - i));
            // Perform matrix multiplication on the GPU
            // This is a placeholder, actual matrix multiplication needs to be implemented using CUDA

            // Transfer results back to GPU after processing each batch
            thrust::copy(uv_points_batch.begin(), uv_points_batch.end(), uv_points_list.begin() + 2 * i);

            // Free GPU memory after processing each batch
            cudaDeviceSynchronize();
        }

        return uv_points_list;
    }

    thrust::device_vector<float> InverseTriangulation::undistorted_points(const thrust::device_vector<float>& points, const Eigen::VectorXd& dist) {
        // Extract distortion coefficients
        float k1 = dist(0);
        float k2 = dist(1);
        float p1 = dist(2);
        float p2 = dist(3);
        float k3 = dist(4);

        // Split points into x and y coordinates
        size_t num_points = points.size() / 2;
        thrust::device_vector<float> x(num_points);
        thrust::device_vector<float> y(num_points);
        thrust::copy(points.begin(), points.begin() + num_points, x.begin());
        thrust::copy(points.begin() + num_points, points.end(), y.begin());

        // Calculate r^2 (squared distance from the origin)
        thrust::device_vector<float> r2(num_points);
        thrust::transform(x.begin(), x.end(), y.begin(), r2.begin(), thrust::plus<float>());

        // Radial distortion
        thrust::device_vector<float> radial(num_points);
        thrust::transform(r2.begin(), r2.end(), radial.begin(), [=] __device__ (float r2_val) {
            return 1 + k1 * r2_val + k2 * r2_val * r2_val + k3 * r2_val * r2_val * r2_val;
        });

        // Tangential distortion
        thrust::device_vector<float> x_tangential(num_points);
        thrust::device_vector<float> y_tangential(num_points);
        thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(x.begin(), y.begin(), r2.begin())),
                          thrust::make_zip_iterator(thrust::make_tuple(x.end(), y.end(), r2.end())),
                          thrust::make_zip_iterator(thrust::make_tuple(x_tangential.begin(), y_tangential.begin())),
                          [=] __device__ (thrust::tuple<float, float, float> t) {
                              float x_val = thrust::get<0>(t);
                              float y_val = thrust::get<1>(t);
                              float r2_val = thrust::get<2>(t);
                              return thrust::make_tuple(2 * p1 * x_val * y_val + p2 * (r2_val + 2 * x_val * x_val),
                                                        p1 * (r2_val + 2 * y_val * y_val) + 2 * p2 * x_val * y_val);
                          });

        // Compute distorted coordinates
        thrust::device_vector<float> x_distorted(num_points);
        thrust::device_vector<float> y_distorted(num_points);
        thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(x.begin(), radial.begin(), x_tangential.begin())),
                          thrust::make_zip_iterator(thrust::make_tuple(x.end(), radial.end(), x_tangential.end())),
                          x_distorted.begin(),
                          [=] __device__ (thrust::tuple<float, float, float> t) {
                              return thrust::get<0>(t) * thrust::get<1>(t) + thrust::get<2>(t);
                          });
        thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(y.begin(), radial.begin(), y_tangential.begin())),
                          thrust::make_zip_iterator(thrust::make_tuple(y.end(), radial.end(), y_tangential.end())),
                          y_distorted.begin(),
                          [=] __device__ (thrust::tuple<float, float, float> t) {
                              return thrust::get<0>(t) * thrust::get<1>(t) + thrust::get<2>(t);
                          });

        // Stack the distorted points
        thrust::device_vector<float> distorted_points(3 * num_points);
        thrust::copy(x_distorted.begin(), x_distorted.end(), distorted_points.begin());
        thrust::copy(y_distorted.begin(), y_distorted.end(), distorted_points.begin() + num_points);
        thrust::fill(distorted_points.begin() + 2 * num_points, distorted_points.end(), 1.0f);

        return distorted_points;
    }

        std::pair<thrust::device_vector<float>, thrust::device_vector<float>> bi_interpolation(const thrust::device_vector<float>& images, const thrust::device_vector<float>& uv_points, int window_size = 3) {
        // Convert images to thrust::device_vector
        thrust::device_vector<float> d_images = images;
        thrust::device_vector<float> d_uv_points = uv_points;

        size_t height = d_images.size() / (width * num_images);
        size_t width = d_images.size() / (height * num_images);
        size_t num_images = d_images.size() / (height * width);

        // Estimate memory usage per point
        size_t memory_per_point = 4 * num_images * 4;
        size_t points_per_batch = std::max<size_t>(1, max_gpu_usage * 1024 * 1024 * 1024 / memory_per_point);

        // Output arrays on GPU
        thrust::device_vector<float> interpolated(num_points * num_images, 0.0f);
        thrust::device_vector<float> std(num_points * num_images, 0.0f);

        for (size_t i = 0; i < num_points; i += points_per_batch) {
            size_t end = std::min(i + points_per_batch, num_points);
            thrust::device_vector<float> uv_batch(d_uv_points.begin() + 2 * i, d_uv_points.begin() + 2 * end);

            // Compute integer and fractional parts of UV coordinates
            thrust::device_vector<int> x(end - i);
            thrust::device_vector<int> y(end - i);
            thrust::transform(uv_batch.begin(), uv_batch.begin() + (end - i), x.begin(), thrust::placeholders::_1);
            thrust::transform(uv_batch.begin() + (end - i), uv_batch.end(), y.begin(), thrust::placeholders::_1);

            thrust::device_vector<int> x1(end - i);
            thrust::device_vector<int> y1(end - i);
            thrust::transform(x.begin(), x.end(), x1.begin(), thrust::placeholders::_1);
            thrust::transform(y.begin(), y.end(), y1.begin(), thrust::placeholders::_1);

            thrust::device_vector<int> x2(end - i);
            thrust::device_vector<int> y2(end - i);
            thrust::transform(x1.begin(), x1.end(), x2.begin(), thrust::placeholders::_1 + 1);
            thrust::transform(y1.begin(), y1.end(), y2.begin(), thrust::placeholders::_1 + 1);

            thrust::device_vector<float> x_diff(end - i);
            thrust::device_vector<float> y_diff(end - i);
            thrust::transform(x.begin(), x.end(), x1.begin(), x_diff.begin(), thrust::minus<float>());
            thrust::transform(y.begin(), y.end(), y1.begin(), y_diff.begin(), thrust::minus<float>());

            for (size_t k = 0; k < num_images; ++k) {
                // Vectorized extraction of corner pixels
                thrust::device_vector<float> p11(end - i);
                thrust::device_vector<float> p12(end - i);
                thrust::device_vector<float> p21(end - i);
                thrust::device_vector<float> p22(end - i);

                // Bilinear interpolation
                thrust::device_vector<float> interpolated_batch(end - i);
                thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(p11.begin(), p21.begin(), p12.begin(), p22.begin(), x_diff.begin(), y_diff.begin())),
                                  thrust::make_zip_iterator(thrust::make_tuple(p11.end(), p21.end(), p12.end(), p22.end(), x_diff.end(), y_diff.end())),
                                  interpolated_batch.begin(),
                                  [=] __device__ (thrust::tuple<float, float, float, float, float, float> t) {
                                      return thrust::get<0>(t) * (1 - thrust::get<4>(t)) * (1 - thrust::get<5>(t)) +
                                             thrust::get<1>(t) * thrust::get<4>(t) * (1 - thrust::get<5>(t)) +
                                             thrust::get<2>(t) * (1 - thrust::get<4>(t)) * thrust::get<5>(t) +
                                             thrust::get<3>(t) * thrust::get<4>(t) * thrust::get<5>(t);
                                  });

                thrust::device_vector<float> std_batch(end - i);
                thrust::transform(thrust::make_zip_iterator(thrust::make_tuple(p11.begin(), p12.begin(), p21.begin(), p22.begin())),
                                  thrust::make_zip_iterator(thrust::make_tuple(p11.end(), p12.end(), p21.end(), p22.end())),
                                  std_batch.begin(),
                                  [=] __device__ (thrust::tuple<float, float, float, float> t) {
                                      return thrust::get<0>(t) + thrust::get<1>(t) + thrust::get<2>(t) + thrust::get<3>(t);
                                  });

                // Store results in GPU arrays
                thrust::copy(interpolated_batch.begin(), interpolated_batch.end(), interpolated.begin() + i * num_images + k);
                thrust::copy(std_batch.begin(), std_batch.end(), std.begin() + i * num_images + k);
            }
        }

        return std::make_pair(interpolated, std);
    }

    std::tuple<thrust::device_vector<float>, thrust::device_vector<float>, thrust::device_vector<float>> InverseTriangulation::spatial_correl(const thrust::device_vector<float>& uv_left, const thrust::device_vector<float>& uv_right, int window_size = 3) {
        size_t half_window = window_size / 2;
        size_t height = left_images.size() / (width * num_images);
        size_t width = left_images.size() / (height * num_images);
        size_t num_images = left_images.size() / (height * width);

        // Estimate memory usage and adjust batch size
        size_t memory_per_point = 2 * window_size * window_size * num_images * 4;  // Approx memory per point
        size_t points_per_batch = std::max<size_t>(1, max_gpu_usage * 1024 * 1024 * 1024 / memory_per_point);

        // Allocate space for results
        thrust::device_vector<float> spatial_corr(num_points, 0.0f);
        thrust::device_vector<float> std_corr(num_points, 0.0f);

        for (size_t i = 0; i < num_points; i += points_per_batch) {
            size_t end_idx = std::min(i + points_per_batch, num_points);
            thrust::device_vector<float> uv_batch_l(uv_left.begin() + 2 * i, uv_left.begin() + 2 * end_idx);
            thrust::device_vector<float> uv_batch_r(uv_right.begin() + 2 * i, uv_right.begin() + 2 * end_idx);

            // Compute integer and fractional parts of UV coordinates
            thrust::device_vector<int> x1_l(end_idx - i);
            thrust::device_vector<int> y1_l(end_idx - i);
            thrust::device_vector<int> x1_r(end_idx - i);
            thrust::device_vector<int> y1_r(end_idx - i);
            thrust::transform(uv_batch_l.begin(), uv_batch_l.begin() + (end_idx - i), x1_l.begin(), thrust::placeholders::_1 - half_window);
            thrust::transform(uv_batch_l.begin() + (end_idx - i), uv_batch_l.end(), y1_l.begin(), thrust::placeholders::_1 - half_window);
            thrust::transform(uv_batch_r.begin(), uv_batch_r.begin() + (end_idx - i), x1_r.begin(), thrust::placeholders::_1 - half_window);
            thrust::transform(uv_batch_r.begin() + (end_idx - i), uv_batch_r.end(), y1_r.begin(), thrust::placeholders::_1 - half_window);

            thrust::device_vector<int> offsets(window_size);
            thrust::sequence(offsets.begin(), offsets.end());

            // Compute indices for batch-based patch extraction
            thrust::device_vector<int> y_indices_l(end_idx - i);
            thrust::device_vector<int> x_indices_l(end_idx - i);
            thrust::device_vector<int> y_indices_r(end_idx - i);
            thrust::device_vector<int> x_indices_r(end_idx - i);
            thrust::transform(y1_l.begin(), y1_l.end(), offsets.begin(), y_indices_l.begin(), thrust::placeholders::_1 + thrust::placeholders::_2);
            thrust::transform(x1_l.begin(), x1_l.end(), offsets.begin(), x_indices_l.begin(), thrust::placeholders::_1 + thrust::placeholders::_2);
            thrust::transform(y1_r.begin(), y1_r.end(), offsets.begin(), y_indices_r.begin(), thrust::placeholders::_1 + thrust::placeholders::_2);
            thrust::transform(x1_r.begin(), x1_r.end(), offsets.begin(), x_indices_r.begin(), thrust::placeholders::_1 + thrust::placeholders::_2);

            // Extract patches for left and right images
            thrust::device_vector<float> patch_left(end_idx - i);
            thrust::device_vector<float> patch_right(end_idx - i);
            // This is a placeholder, actual patch extraction needs to be implemented using CUDA

            // Compute mean subtraction
            thrust::device_vector<float> mean_left(end_idx - i);
            thrust::device_vector<float> std_left(end_idx - i);
            thrust::device_vector<float> mean_right(end_idx - i);
            thrust::device_vector<float> std_right(end_idx - i);
            // This is a placeholder, actual mean and std computation needs to be implemented using CUDA

            thrust::device_vector<float> comb_std_left(end_idx - i);
            thrust::device_vector<float> comb_std_right(end_idx - i);
            thrust::transform(std_left.begin(), std_left.end(), comb_std_left.begin(), thrust::placeholders::_1);
            thrust::transform(std_right.begin(), std_right.end(), comb_std_right.begin(), thrust::placeholders::_1);

            thrust::transform(patch_left.begin(), patch_left.end(), mean_left.begin(), patch_left.begin(), thrust::minus<float>());
            thrust::transform(patch_right.begin(), patch_right.end(), mean_right.begin(), patch_right.begin(), thrust::minus<float>());

            // Compute spatial correlation
            thrust::device_vector<float> num(end_idx - i);
            thrust::device_vector<float> den_left(end_idx - i);
            thrust::device_vector<float> den_right(end_idx - i);
            thrust::transform(patch_left.begin(), patch_left.end(), patch_right.begin(), num.begin(), thrust::multiplies<float>());
            thrust::transform(patch_left.begin(), patch_left.end(), patch_left.begin(), den_left.begin(), thrust::multiplies<float>());
            thrust::transform(patch_right.begin(), patch_right.end(), patch_right.begin(), den_right.begin(), thrust::multiplies<float>());

            thrust::transform(comb_std_left.begin(), comb_std_left.end(), comb_std_right.begin(), std_corr.begin() + i, thrust::plus<float>());
            thrust::transform(num.begin(), num.end(), den_left.begin(), den_right.begin(), spatial_corr.begin() + i, thrust::divides<float>());

            // Clear variables to manage memory
            cudaDeviceSynchronize();
        }

        // Reshape and compute maximum correlations
        thrust::device_vector<float> reshaped_corr(num_points);
        thrust::copy(spatial_corr.begin(), spatial_corr.end(), reshaped_corr.begin());
        thrust::device_vector<float> spatial_max(num_points);
        thrust::device_vector<float> spatial_id(num_points);
        thrust::device_vector<bool> correl_mask = create_correl_mask(std_corr, spatial_max, threshold, 60);
        thrust::transform(reshaped_corr.begin(), reshaped_corr.end(), spatial_id.begin(), thrust::placeholders::_1);

        return std::make_tuple(spatial_id, spatial_max, std_corr);
    }

        thrust::device_vector<float> InverseTriangulation::correlation_process(const thrust::device_vector<float>& points_3d, int win_size = 3, float threshold = 0.8, bool save_points = true, bool visualize = false) {
        auto t0 = std::chrono::high_resolution_clock::now();
        thrust::device_vector<float> uv_left = transform_gcs2ccs(points_3d, "left");
        thrust::device_vector<float> uv_right = transform_gcs2ccs(points_3d, "right");

        auto t1 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = t1 - t0;
        std::cout << "Transform points to image: " << elapsed.count() << " s" << std::endl;

        auto t2 = std::chrono::high_resolution_clock::now();
        auto [spatial_id, spatial_max, std_corr] = spatial_correl(uv_left, uv_right, win_size);
        auto t3 = std::chrono::high_resolution_clock::now();
        elapsed = t3 - t2;
        std::cout << "Spatial correlation time: " << elapsed.count() << " s" << std::endl;

        thrust::device_vector<bool> correl_mask = correl_mask(std_corr, spatial_max, threshold, 60);
        thrust::device_vector<float> space_temp_correl_pt(points_3d.size() / 3);
        thrust::copy_if(points_3d.begin(), points_3d.end(), correl_mask.begin(), space_temp_correl_pt.begin(), thrust::identity<bool>());

        auto t4 = std::chrono::high_resolution_clock::now();
        elapsed = t4 - t0;
        std::cout << "Correlation process: " << elapsed.count() << " s" << std::endl;

        if (save_points) {
            save_points(space_temp_correl_pt, "./sm3_tubo.csv");
        }
        if (visualize) {
            plot_3d_points(space_temp_correl_pt, spatial_max, "Temporal x Spatial correlation result\n" + std::to_string(right_images.size() / (height * width)) + " imgs\n" + std::to_string(win_size) + " win size");
        }

        return space_temp_correl_pt;
    }