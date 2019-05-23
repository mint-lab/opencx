/**
 * An example of CVX
 *
 * @author  Sunglok Choi (http://sites.google.com/site/sunglok)
 * @version 0.2 (05/23/2019)
 */

#include "opencvx.hpp"
#include <iostream>

using namespace std;

// An example to use 'cvx::Algorithm'
class NoiseGenerator : public cvx::Algorithm
{
public:
    NoiseGenerator() : m_rng(cv::getTickCount())
    {
        m_name = "Normal";
        m_uniform_range = (0, 0);
        m_gauss_mean = 0;
        m_gauss_stdev = 1;
    }

    // After implementing 'readParam()', it is possible to configure internal parameters without defining their individual setters.
    virtual int readParam(const cv::FileNode& fn)
    {
        int n_read = cvx::Algorithm::readParam(fn);
        CVX_LOAD_PARAM_COUNT(fn, "name", m_name, n_read);                   // Default: "Normal"
        CVX_LOAD_PARAM_COUNT(fn, "uniform_range", m_uniform_range, n_read); // Default: (0, 0)
        CVX_LOAD_PARAM_COUNT(fn, "gaussian_mu", m_gauss_mean, n_read);      // Default: 0
        CVX_LOAD_PARAM_COUNT(fn, "gaussian_sigma", m_gauss_stdev, n_read);  // Default: 1
        return n_read;
    }

    // After implementing 'writeParam()' and 'readParam()', it is possible to save and load the internal parameters in a file.
    virtual bool writeParam(cv::FileStorage& fs) const
    {
        if (cvx::Algorithm::writeParam(fs))
        {
            fs << "name" << m_name;
            fs << "uniform_range" << m_uniform_range;
            fs << "gaussian_mu" << m_gauss_mean;
            fs << "gaussian_sigma" << m_gauss_stdev;
            return true;
        }
        return false;
    }

    double getNoise() { return m_rng.uniform(m_uniform_range(0), m_uniform_range(1)) + m_gauss_mean + m_rng.gaussian(m_gauss_stdev); }

    string getName() { return m_name; }

protected:
    cv::RNG m_rng;

    string m_name;

    cv::Vec2d m_uniform_range;

    double m_gauss_mean;

    double m_gauss_stdev;
};

pair<double, double> calcMeanVar(NoiseGenerator& generator, int n)
{
    double sum = 0, squared_sum = 0;
    for (int i = 0; i < n; i++)
    {
        double noise = generator.getNoise();
        sum += noise;
        squared_sum += noise * noise;
    }
    double avg = sum / n;
    double var = squared_sum / n - sum * sum / n / n;
    return make_pair(avg, var);
}

int testAlgorithm(const char* filename = "noise_generator.yml", int n = 1000)
{
    cout << "### Test cvx::Algorithm" << endl;
    NoiseGenerator generator;

    // Test #1
    auto test1 = calcMeanVar(generator, n);
    cout << "* Name: " << generator.getName() << endl;
    cout << "  * Mean: " << test1.first << endl;
    cout << "  * Variance: " << test1.second << endl;

    // Test #2 after setting parameters
    if (!generator.setParam("name: Gaussian (mu = 4, sigma = 2")) return -1;
    if (!generator.setParamValue("gaussian_mu", 4)) return -1;
    if (!generator.setParamValue("gaussian_sigma", 2)) return -1;
    auto test2 = calcMeanVar(generator, n);
    cout << "* Name: " << generator.getName() << endl;
    cout << "  * Mean: " << test2.first << endl;
    cout << "  * Variance: " << test2.second << endl;

    // Test #3 after setting parameters
    if (!generator.setParamTexts("name", "Normal Again")) return -1;
    if (!generator.setParam("gaussian_mu: 0\ngaussian_sigma: 1")) return -1;
    auto test3 = calcMeanVar(generator, n);
    cout << "* Name: " << generator.getName() << endl;
    cout << "  * Mean: " << test3.first << endl;
    cout << "  * Variance: " << test3.second << endl;

    // Test #4 after setting parameters
    if (!generator.setParamTexts("name", "Normal + Uniform(10, 18)")) return -1;
    if (!generator.setParamValue("uniform_range", { 10, 18 })) return -1;
    auto test4 = calcMeanVar(generator, n);
    cout << "* Name: " << generator.getName() << endl;
    cout << "  * Mean: " << test4.first << endl;
    cout << "  * Variance: " << test4.second << endl;
    if (!generator.saveParam(filename)) return -1;

    // Test #5 after setting parameters
    if (!generator.setParamTexts("name", "Normal Again and Again")) return -1;
    if (!generator.setParam("uniform_range: [ 0, 0 ]")) return -1;
    auto test5 = calcMeanVar(generator, n);
    cout << "* Name: " << generator.getName() << endl;
    cout << "  * Mean: " << test5.first << endl;
    cout << "  * Variance: " << test5.second << endl;

    // Test #6 after loading parameters
    if (!generator.loadParam(filename)) return -1;
    auto test6 = calcMeanVar(generator, n);
    cout << "* Name: " << generator.getName() << endl;
    cout << "  * Mean: " << test6.first << endl;
    cout << "  * Variance: " << test6.second << endl;

    return 0;
}

int compareVideoWriter(const string postfix = "example.avi", int frame = 100, double fps = 10)
{
    // Generate sample images
    vector<cv::Mat> video;
    for (int f = 0; f < frame; f++)
    {
        cv::Mat image(640, 480, CV_8UC3);
        uchar v = uchar(255 * f / (frame - 1));
        image = cv::Vec3b(0, v, 0);
        video.push_back(image);
    }

    // Make a video file using 'cv::VideoWriter' which needs the size and channel of images in advance
    cv::VideoWriter cv_writer;
    if (!cv_writer.open("cv_" + postfix, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(640, 480), true)) return -1;
    for (auto img = video.begin(); img != video.end(); img++) cv_writer << *img; 
    // Make a video file using 'cvx::VideoWriter'
    cvx::VideoWriter cvx_writer;
    if (!cvx_writer.open("cvx_" + postfix, fps)) return -1;
    for (auto img = video.begin(); img != video.end(); img++)
        cvx_writer << *img;

    return 0;
}

int testAngularOperations()
{
    cout << "### Angular Conversion" << endl;
    cout << "* CV_PI / 2 [rad] == " << cvx::cvtRad2Deg(CV_PI / 2) << " [deg]." << endl;
    cout << "*        90 [deg] == CV_PI / " << CV_PI / cvx::cvtDeg2Rad(90) << " [rad]." << endl;
    cout << endl;
    cout << "### Angular Trimming" << endl;
    cout << "*    2 * CV_PI [rad] == " << cvx::trimRad(2 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*  1.5 * CV_PI [rad] == " << cvx::trimRad(1.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*        CV_PI [rad] == " << cvx::trimRad(CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*  0.5 * CV_PI [rad] == " << cvx::trimRad(0.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*            0 [rad] == " << cvx::trimRad(0) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "* -0.5 * CV_PI [rad] == " << cvx::trimRad(-0.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*       -CV_PI [rad] == " << cvx::trimRad(-CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "* -1.5 * CV_PI [rad] == " << cvx::trimRad(-1.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*   -2 * CV_PI [rad] == " << cvx::trimRad(-2 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << endl;
    return 0;
}

int testKeyCodes()
{
    cout << "### Key Codes for cv::waitKeyEx()" << endl;
    while (true)
    {
        cv::imshow("Key Codes for cv::waitKeyEx()", cv::Mat::zeros(320, 240, CV_8U));
        int key = cv::waitKeyEx();
        cout << "* Key code " << key;
        if (key == cvx::KEY_LF) cout << " is defined as 'cvx::KEY_LF'." << endl;
        else if (key == cvx::KEY_CR) cout << " is defined as 'cvx::KEY_CR'." << endl;
        else if (key == cvx::KEY_TAB) cout << " is defined as 'cvx::KEY_TAB'." << endl;
        else if (key == cvx::KEY_ESC) cout << " is defined as 'cvx::KEY_ESC'." << endl;
        else if (key == cvx::KEY_SPACE) cout << " is defined as 'cvx::KEY_SPACE'." << endl;
        else if (key == cvx::KEY_UP) cout << " is defined as 'cvx::KEY_UP'." << endl;
        else if (key == cvx::KEY_DOWN) cout << " is defined as 'cvx::KEY_DOWN'." << endl;
        else if (key == cvx::KEY_LEFT) cout << " is defined as 'cvx::KEY_LEFT'." << endl;
        else if (key == cvx::KEY_RIGHT) cout << " is defined as 'cvx::KEY_RIGHT'." << endl;
        else cout << " is not defined." << endl;

        if (key == cvx::KEY_ESC) break;
    }
    return 0;
}

int main(void)
{
    return testAlgorithm();
    compareVideoWriter();
    testAngularOperations();
    testKeyCodes();
    return 0;
}