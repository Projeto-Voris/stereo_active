# Stereo Active Package

This package provides functionalities for stereo image acquisition, noise image generation, and inverse triangulation using ROS 2. It includes nodes for capturing stereo images, displaying noise images, and performing inverse triangulation to generate 3D point clouds.

## Features

- **Stereo Image Acquisition**: Captures synchronized stereo images and saves them to a specified directory.
- **Noise Image Generation**: Generates and displays noise images on a specified monitor.
- **Inverse Triangulation**: Processes stereo images to generate 3D point clouds using inverse triangulation.

## Dependencies

- ROS 2
- OpenCV
- Eigen
- CUDA
- Thrust
- libnoise
- YAML-CPP

## Installation

1. **Clone the repository**:
    ```sh
    git clone https://github.com/yourusername/stereo_active.git
    cd stereo_active
    ```

2. **Install libnoise**:
    ```sh
    git clone https://github.com/qknight/libnoise.git
    cd libnoise
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig
    ```

3. **Build the package**:
    ```sh
    cd ~/ros2_ws
    colcon build
    ```

## Usage

### Launching Nodes

1. **Stereo Image Acquisition**:
    ```sh
    ros2 launch stereo_active stereo_acquisition.launch.py
    ```

2. **Noise Image Generation**:
    ```sh
    ros2 launch stereo_active noise_display.launch.py
    ```

3. **Inverse Triangulation**:
    ```sh
    ros2 launch stereo_active inverse_triangulation.launch.py
    ```


## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](http://_vscodecontentref_/0) file for details.

## Acknowledgments

- [libnoise](https://github.com/qknight/libnoise) for noise generation.