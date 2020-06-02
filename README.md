## OpenCX: Sunglok's OpenCV Extension
_OpenCX_ aims to provide extended functionality and tools to [OpenCV](https://opencv.org/) for more convenience. It consists of several header files, `opencx.hpp` and others, which only depend on OpenCV in C++. Just include the file to your project. It will work without complex configuration and dependency. OpenCX is [Beerware](http://en.wikipedia.org/wiki/Beerware) so that it is free to use and distribute.

* Homepage: <https://github.com/sunglok/opencx>

### File Description
* `opencx.hpp`: My OpenCV extension
* `opensx.hpp`: My C++ extension not depending on OpenCV
* `ekf.hpp`: My implementation of the [extended Kalman filter](http://en.wikipedia.org/wiki/Extended_Kalman_filter)
* `example_cx.cpp`: Examples using `opencx.hpp`
* `example_sx.cpp`: Examples using `opensx.hpp`
* `example_ekf.cpp`: Pose estimation examples using `cv::KalmanFilter` and `cx::EKF`
* `README.md`: A brief introduction to OpenCX
* `CMakeList.txt`: A [CMake](https://cmake.org/) script to build examples

### Running Examples
1. `git clone https://github.com/sunglok/opencx.git`: Clone OpenCX repository
2. `mkdir opecx/build && cd opencx/build`: Make a build directory
3. `cmake ..`: Prepare to build (generating `Makefile` file or [MSVS](https://visualstudio.microsoft.com/) solution/project files)
    * In Windows, you need to specify the location of OpenCV (where `OpenCVConfig.cmake` is exist) as follows: `cmake .. -D OpenCV_DIR:PATH="C:\Your\OpenCV\Dir"`.
    * You can use [cmake-gui](https://cmake.org/runningcmake/) for easier CMake configuration.
4. `make`: Build examples
    * In Windows, please open `opencx.sln` and build projects.

### License
* [Beerware](http://en.wikipedia.org/wiki/Beerware)

### Contact
* [Sunglok Choi](http://sites.google.com/site/sunglok/) (sunglok AT hanmail DOT net)
