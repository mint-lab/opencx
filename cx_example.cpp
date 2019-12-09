/**
 * An example of OpenCX
 *
 * @author  Sunglok Choi (http://sites.google.com/site/sunglok)
 * @version 0.2 (05/23/2019)
 */

#include "opencx.hpp"
#include <iostream>

using namespace std;

// An example to use 'cx::Algorithm'
class NoiseGenerator : public cx::Algorithm
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
        int n_read = cx::Algorithm::readParam(fn);
        CX_LOAD_PARAM_COUNT(fn, "name", m_name, n_read);                    // Default: "Normal"
        CX_LOAD_PARAM_COUNT(fn, "uniform_range", m_uniform_range, n_read);  // Default: (0, 0)
        CX_LOAD_PARAM_COUNT(fn, "gaussian_mu", m_gauss_mean, n_read);       // Default: 0
        CX_LOAD_PARAM_COUNT(fn, "gaussian_sigma", m_gauss_stdev, n_read);   // Default: 1
        return n_read;
    }

    // After implementing 'writeParam()' and 'readParam()', it is possible to save and load the internal parameters in a file.
    virtual bool writeParam(cv::FileStorage& fs) const
    {
        if (!cx::Algorithm::writeParam(fs)) return false;
        fs << "name" << m_name;
        fs << "uniform_range" << m_uniform_range;
        fs << "gaussian_mu" << m_gauss_mean;
        fs << "gaussian_sigma" << m_gauss_stdev;
        return true;
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
    cout << "### Test cx::Algorithm" << endl;
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

int compareVideoWriter(const string& postfix = "example.avi", int frame = 100, double fps = 10)
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

    // 1) Make a video file using 'cv::VideoWriter' which needs the size and channel of images in advance
    cv::VideoWriter cv_writer;
    if (!cv_writer.open("cv_" + postfix, cv::VideoWriter::fourcc('X', 'V', 'I', 'D'), fps, cv::Size(640, 480), true)) return -1;
    for (auto img = video.begin(); img != video.end(); img++) cv_writer << *img; 

    // 2) Make a video file using 'cx::VideoWriter'
    cx::VideoWriter cx_writer;
    if (!cx_writer.open("cx_" + postfix, fps)) return -1;
    for (auto img = video.begin(); img != video.end(); img++)
        cx_writer << *img;

    return 0;
}

int testAngularOperations()
{
    cout << "### Angular Conversion" << endl;
    cout << "* CV_PI / 2 [rad] == " << cx::cvtRad2Deg(CV_PI / 2) << " [deg]." << endl;
    cout << "*        90 [deg] == CV_PI / " << CV_PI / cx::cvtDeg2Rad(90) << " [rad]." << endl;
    cout << endl;
    cout << "### Angular Trimming" << endl;
    cout << "*    2 * CV_PI [rad] == " << cx::trimRad(2 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*  1.5 * CV_PI [rad] == " << cx::trimRad(1.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*        CV_PI [rad] == " << cx::trimRad(CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*  0.5 * CV_PI [rad] == " << cx::trimRad(0.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*            0 [rad] == " << cx::trimRad(0) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "* -0.5 * CV_PI [rad] == " << cx::trimRad(-0.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*       -CV_PI [rad] == " << cx::trimRad(-CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "* -1.5 * CV_PI [rad] == " << cx::trimRad(-1.5 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << "*   -2 * CV_PI [rad] == " << cx::trimRad(-2 * CV_PI) / CV_PI << " * CV_PI [rad]." << endl;
    cout << endl;
    return 0;
}

int testLowerCase(const cv::String& text = "OpenCX makes OpenCV v4 easier!")
{
    cout << "### String toLowerCase" << endl;
    cout << "* Original text: " << text << endl;
    cout << "* Transformed text: " << cx::toLowerCase(text) << endl;
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
        if (key == cx::KEY_LF) cout << " is defined as 'cx::KEY_LF'." << endl;
        else if (key == cx::KEY_CR) cout << " is defined as 'cx::KEY_CR'." << endl;
        else if (key == cx::KEY_TAB) cout << " is defined as 'cx::KEY_TAB'." << endl;
        else if (key == cx::KEY_ESC) cout << " is defined as 'cx::KEY_ESC'." << endl;
        else if (key == cx::KEY_SPACE) cout << " is defined as 'cx::KEY_SPACE'." << endl;
        else cout << " is not defined." << endl;

        if (key == cx::KEY_ESC) break;
    }
    return 0;
}

int main()
{
    testAlgorithm();
    compareVideoWriter();
    testAngularOperations();
    testLowerCase();
    testKeyCodes();
    return 0;
}
