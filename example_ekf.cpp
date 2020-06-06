/**
 * An example of OpenCX, especially for 'ekf.hpp'
 */

#include "opencx.hpp"
#include <iostream>

using namespace std;

// An interface for 2D pose and velocity estimation from GPS data
class GPSLocalizer
{
public:
    virtual ~GPSLocalizer() { } 
    virtual bool applyGPS(const cv::Vec2d& gps, double time) = 0;
    virtual cv::Vec3d getPose() const = 0;
    virtual cv::Vec2d getVelocity() const = 0;
    virtual bool setControlNoise(double noise) = 0;
    virtual bool setGPSNoise(const cv::Vec2d& noise, const cv::Vec2d& offset) = 0;
};

// GPS localization using 'cx::EKF'
// - The state variable: $\mathbf{x} = [ x, y, \theta, v, w ]^\top$
// - The state transition model: The constant velocity model
// - The observation model: GPS data with off-centered installation
// - Reference: Choi and Kim, Leveraging Localization Accuracy with Off-centered GPS, IEEE T-ITS, Vol. 21, No. 6, 2020 <a href="http://rit.kaist.ac.kr/home/International_Journal?action=AttachFile&do=get&target=paper_0425.pdf">PDF</a> <a href="http://doi.org/10.1109/TITS.2019.2915108">DOI</a>
class GPSLocalizerCx : public GPSLocalizer, public cx::EKF
{
public:
    GPSLocalizerCx()
    {
        initialize(cv::Mat::zeros(5, 1, CV_64F), cv::Mat::eye(5, 5, CV_64F));
        m_ctrl_noise = 1;
        m_gps_noise = cv::Mat::eye(2, 2, CV_64F);
        m_prev_time = -1;
    }

    virtual bool applyGPS(const cv::Vec2d& gps, double timestamp)
    {
        double dt = 0;
        if (m_prev_time > 0) dt = timestamp - m_prev_time;
        if (dt > DBL_EPSILON) predict(dt);
        m_prev_time = timestamp;
        return correct(gps);
    }

    virtual cv::Vec3d getPose() const { return m_state_vec.rowRange(0, 3); }

    virtual cv::Vec2d getVelocity() const { return m_state_vec.rowRange(3, 5); }

    virtual bool setControlNoise(double noise)
    {
        m_ctrl_noise = noise;
        return true;
    }

    virtual bool setGPSNoise(const cv::Vec2d& noise, const cv::Vec2d& offset = cv::Vec2d(0, 0))
    {
        m_gps_noise = 0;
        m_gps_noise.at<double>(0, 0) = noise(0) * noise(0);
        m_gps_noise.at<double>(1, 1) = noise(1) * noise(1);
        m_gps_offset = offset;
        return true;
    }

protected:
    virtual cv::Mat transitFunc(const cv::Mat& state, const cv::Mat& control, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double dt = control.at<double>(0);
        const double vt = state.at<double>(3) * dt;
        const double wt = state.at<double>(4) * dt;
        const double c = cos(state.at<double>(2) + wt / 2), s = sin(state.at<double>(2) + wt / 2);
        cv::Mat func = (cv::Mat_<double>(5, 1) <<
            state.at<double>(0) + vt * c,
            state.at<double>(1) + vt * s,
            state.at<double>(2) + wt,
            state.at<double>(3),
            state.at<double>(4));
        jacobian = (cv::Mat_<double>(5, 5) <<
            1, 0, -vt * s, dt * c, -vt * dt * s / 2,
            0, 1,  vt * c, dt * s,  vt * dt * c / 2,
            0, 0, 1, 0, dt,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1);
        cv::Mat W = (cv::Mat_<double>(5, 1) <<
            state.at<double>(3) * c - vt * s,
            state.at<double>(3) * s + vt * c,
            state.at<double>(4),
            DBL_EPSILON,
            DBL_EPSILON);
        noise = W * m_ctrl_noise * W.t();
        return func;
    }

    virtual cv::Mat observeFunc(const cv::Mat& state, const cv::Mat& measure, cv::Mat& jacobian, cv::Mat& noise)
    {
        const double c = cos(state.at<double>(2) + m_gps_offset(1)), s = sin(state.at<double>(2) + m_gps_offset(1));
        cv::Mat func = (cv::Mat_<double>(2, 1) <<
            state.at<double>(0) + m_gps_offset(0) * c,
            state.at<double>(1) + m_gps_offset(0) * s);
        jacobian = (cv::Mat_<double>(2, 5) <<
            1, 0, -m_gps_offset(0) * s, 0, 0,
            0, 1,  m_gps_offset(0) * c, 0, 0);
        noise = m_gps_noise;
        return func;
    }

    double m_ctrl_noise;

    cv::Mat m_gps_noise;

    cv::Vec2d m_gps_offset;

    double m_prev_time;
};

// GPS localization using 'cv::KalmanFilter' in OpenCV
// - Same state and models with 'GPSLocalizerCx'. Please compare both implementation how to apply prediction and correction.
class GPSLocalizerCv : public GPSLocalizer, public cv::KalmanFilter
{
public:
    GPSLocalizerCv()
    {
        init(5, 2, 1, CV_64F); // The dimension of state, measurement, and control
        statePre = cv::Mat::zeros(5, 1, CV_64F);
        statePost = cv::Mat::zeros(5, 1, CV_64F);
        errorCovPre = cv::Mat::eye(5, 5, CV_64F);
        errorCovPost = cv::Mat::eye(5, 5, CV_64F);
        transitionMatrix = cv::Mat::eye(3, 3, CV_64F);
        processNoiseCov = cv::Mat::eye(3, 3, CV_64F);
        measurementMatrix = cv::Mat::eye(2, 3, CV_64F);
        measurementNoiseCov = cv::Mat::eye(2, 2, CV_64F);

        m_ctrl_noise = 1;
        m_prev_time = -1;
    }

    virtual bool applyGPS(const cv::Vec2d& gps, double timestamp)
    {
        double dt = 0;
        if (m_prev_time > 0) dt = timestamp - m_prev_time;
        if (dt > DBL_EPSILON)
        {
            // Predict the state
            const double vt = statePost.at<double>(3) * dt;
            const double wt = statePost.at<double>(4) * dt;
            const double c = cos(statePost.at<double>(2) + wt / 2), s = sin(statePost.at<double>(2) + wt / 2);
            transitionMatrix = (cv::Mat_<double>(5, 5) <<
                1, 0, -vt * s, dt * c, -vt * dt * s / 2,
                0, 1,  vt * c, dt * s,  vt * dt * c / 2,
                0, 0, 1, 0, dt,
                0, 0, 0, 1, 0,
                0, 0, 0, 0, 1);
            cv::Mat W = (cv::Mat_<double>(5, 1) <<
                statePost.at<double>(3) * c - vt * s,
                statePost.at<double>(3) * s + vt * c,
                statePost.at<double>(4),
                DBL_EPSILON,
                DBL_EPSILON);
            processNoiseCov = W * m_ctrl_noise * W.t();
            predict();
            statePre = (cv::Mat_<double>(5, 1) <<
                statePost.at<double>(0) + vt * c,
                statePost.at<double>(1) + vt * s,
                statePost.at<double>(2) + wt,
                statePost.at<double>(3),
                statePost.at<double>(4));
        }
        m_prev_time = timestamp;

        // Correct the state
        const double c = cos(statePre.at<double>(2) + m_gps_offset(1)), s = sin(statePre.at<double>(2) + m_gps_offset(1));
        measurementMatrix = (cv::Mat_<double>(2, 5) <<
            1, 0, -m_gps_offset(0) * s, 0, 0,
            0, 1,  m_gps_offset(0) * c, 0, 0);
        // 'measurementNoiseCov' is constant, so it is not necessary to calculate again.
        correct(cv::Mat(gps));
        cv::Mat expectation = (cv::Mat_<double>(2, 1) <<
            statePre.at<double>(0) + m_gps_offset(0) * c,
            statePre.at<double>(1) + m_gps_offset(0) * s);
        temp5 = cv::Mat(gps) - expectation;
        statePost = statePre + gain * temp5;
        return true;
    }

    virtual cv::Vec3d getPose() const { return statePost.rowRange(0, 3); }

    virtual cv::Vec2d getVelocity() const { return statePost.rowRange(3, 5); }

    virtual bool setControlNoise(double noise)
    {
        m_ctrl_noise = noise;
        return true;
    }

    virtual bool setGPSNoise(const cv::Vec2d& noise, const cv::Vec2d& offset = cv::Vec2d(0, 0))
    {
        measurementNoiseCov = 0;
        measurementNoiseCov.at<double>(0, 0) = noise(0) * noise(0);
        measurementNoiseCov.at<double>(1, 1) = noise(1) * noise(1);
        m_gps_offset = offset;
        return true;
    }

protected:
    double m_ctrl_noise;

    cv::Vec2d m_gps_offset;

    double m_prev_time;
};

int testGPSLocalization(GPSLocalizer* localizer, double gps_noise = 0.3, const cv::Vec2d gps_offset = cv::Vec2d(1, 0), double interval = 0.1, double velocity = 1)
{
    if (localizer == nullptr) return -1;
    if (!localizer->setGPSNoise({ gps_noise, gps_noise }, gps_offset)) return -1;

    printf("| Time [sec] | GPS Data [m] | Pose [m] [deg] | Velocity [m/s] [deg/s] |\n");
    printf("| ---------- | ------------ | -------------- | ---------------------- |\n");
    for (double t = interval; t < 10; t += interval)
    {
        cv::Vec3d truth(velocity * t, 1, 0); // Going straight from (0, 1, 0)

        // Simulate and apply noisy GPS data
        cv::Vec2d gps(truth(0), truth(1));
        gps(0) += gps_offset(0) * cos(truth(2) + gps_offset(1)) + cv::theRNG().gaussian(gps_noise);
        gps(1) += gps_offset(0) * sin(truth(2) + gps_offset(1)) + cv::theRNG().gaussian(gps_noise);
        if (!localizer->applyGPS(gps, t)) return -1;

        // Print the current pose
        cv::Vec3d pose = localizer->getPose();
        cv::Vec2d velocity = localizer->getVelocity();
        printf("| %.1f | %.3f, %.3f | %.3f, %.3f, %.1f | %.3f, %.1f |\n", t, gps(0), gps(1), pose(0), pose(1), cx::cvtRad2Deg(pose(2)), velocity(0), cx::cvtRad2Deg(velocity(1)));
    }
    return 0;
}

int main()
{
    printf("\n### Test GPSLocalizerCx with cx::EKF\n");
    GPSLocalizerCx localizer_cx;
    if (testGPSLocalization(&localizer_cx) < 0) return -1;

    printf("\n### Test GPSLocalizerCv with cv::KalmanFilter\n");
    GPSLocalizerCv localizer_cv;
    if (testGPSLocalization(&localizer_cv) < 0) return -1;

    return 0;
}
