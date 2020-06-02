/**
 * An example of OpenCX, especially for 'ekf.hpp'
 */

#include "opencx.hpp"
#include <iostream>

using namespace std;

// An interface for 2D pose estimation
// - The state variable: $\mathbf{x} = [ x, y, \theta ]^\top$
// - The state transition model with the control input $\mathbf{u} = [ \rho, \phi ]^\top$
//   $x_k = x_{k-1} + \rho \cos(\theta_k + \phi / 2)$
//   $y_k = y_{k-1} + \rho \sin(\theta_k + \phi / 2)$
//   $\theta_k = \theta_{k-1} + \phi$
// - The observation model of the measurement $\mathbf{z} = [ x_{GPS}, y_{GPS} ]^\top$
//   $x_{GPS} = x_k$
//   $y_{GPS} = y_k$
class PoseEstimator
{
public:
    virtual ~PoseEstimator() { } 
    virtual bool initPose(const cv::Vec3d& pose, const cv::Vec3d& var) = 0;
    virtual cv::Vec3d getPose() const = 0;
    virtual bool applyOdometry(const cv::Vec2d& delta) = 0;
    virtual bool applyGPS(const cv::Vec2d& gps) = 0;
    virtual bool setControlNoise(const cv::Vec2d& noise) = 0;
    virtual bool setObserveNoise(const cv::Vec2d& noise) = 0;
};

// Pose estimator_cx using 'cx::EKF'
class PoseEstimatorCx : public PoseEstimator, public cx::EKF
{
public:
    PoseEstimatorCx()
    {
        initPose();
        m_ctrl_noise = cv::Mat::eye(2, 2, CV_64F);
        m_obsv_noise = cv::Mat::eye(2, 2, CV_64F);
    }

    virtual bool initPose(const cv::Vec3d& pose = cv::Vec3d(0, 0, 0), const cv::Vec3d& var = cv::Vec3d(1, 1, 1)) { return initialize(pose, cv::Mat::diag(cv::Mat(var))); }

    virtual cv::Vec3d getPose() const { return m_state_vec; }

    virtual bool applyOdometry(const cv::Vec2d& delta) { return predict(delta); }

    virtual bool applyGPS(const cv::Vec2d& gps) { return correct(gps); }

    virtual bool setControlNoise(const cv::Vec2d& noise)
    {
        m_ctrl_noise = 0;
        m_ctrl_noise.at<double>(0, 0) = noise(0) * noise(0);
        m_ctrl_noise.at<double>(1, 1) = noise(1) * noise(1);
        return true;
    }

    virtual bool setObserveNoise(const cv::Vec2d& noise)
    {
        m_obsv_noise = 0;
        m_obsv_noise.at<double>(0, 0) = noise(0) * noise(0);
        m_obsv_noise.at<double>(1, 1) = noise(1) * noise(1);
        return true;
    }

protected:
    virtual cv::Mat transitModel(const cv::Mat& state, const cv::Mat& control)
    {
        double angle = state.at<double>(2) + control.at<double>(1) / 2;
        return (cv::Mat_<double>(3, 1) <<
            state.at<double>(0) + control.at<double>(0) * cos(angle),
            state.at<double>(1) + control.at<double>(0) * sin(angle),
            state.at<double>(2) + control.at<double>(1));
    }

    virtual cv::Mat transitJacobian(const cv::Mat& state, const cv::Mat& control)
    {
        double angle = state.at<double>(2) + control.at<double>(1) / 2;
        return (cv::Mat_<double>(3, 3) <<
            1, 0, control.at<double>(0) * -sin(angle),
            0, 1, control.at<double>(0) *  cos(angle),
            0, 0, 1);
    }

    virtual cv::Mat transitNoiseCov(const cv::Mat& state, const cv::Mat& control)
    {
        double angle = state.at<double>(2) + control.at<double>(1) / 2;
        double c = cos(angle), s = sin(angle);
        cv::Mat W = (cv::Mat_<double>(3, 2) <<
            c, -control.at<double>(0) * s,
            s,  control.at<double>(0) * c,
            0, 1);
        return W * m_ctrl_noise * W.t();
    }

    virtual cv::Mat observeModel(const cv::Mat& state, const cv::Mat& measurement) { return m_state_vec.rowRange(cv::Range(0, 2)); }

    virtual cv::Mat observeJacobian(const cv::Mat& state, const cv::Mat& measurement) { return cv::Mat::eye(2, 3, m_state_vec.type()); };

    virtual cv::Mat observeNoiseCov(const cv::Mat& state, const cv::Mat& measurement) { return m_obsv_noise; }

    cv::Mat m_ctrl_noise;
    cv::Mat m_obsv_noise;
};

// Pose estimator_cx using 'cv::KalmanFilter' in OpenCV
class PoseEstimatorCv : public PoseEstimator, public cv::KalmanFilter
{
public:
    PoseEstimatorCv()
    {
        initPose();
        transitionMatrix = cv::Mat::eye(3, 3, CV_64F);
        processNoiseCov = cv::Mat::eye(3, 3, CV_64F);
        measurementMatrix = cv::Mat::eye(2, 3, CV_64F);
        measurementNoiseCov = cv::Mat::eye(2, 2, CV_64F);
        m_ctrl_noise = cv::Mat::eye(2, 2, CV_64F);
    }

    virtual bool initPose(const cv::Vec3d& pose = cv::Vec3d(0, 0, 0), const cv::Vec3d& var = cv::Vec3d(1, 1, 1))
    {
        init(3, 2, 2, CV_64F); // The dimension of state, measurement, and control
        statePre = pose;
        statePost = pose;
        errorCovPre = (cv::Mat_<double>(3, 3) << var(0), 0, 0, 0, var(1), 0, 0, 0, var(2));
        errorCovPost = (cv::Mat_<double>(3, 3) << var(0), 0, 0, 0, var(1), 0, 0, 0, var(2));
        return true;
    }

    virtual cv::Vec3d getPose() const { return statePre; }

    virtual bool applyOdometry(const cv::Vec2d& delta)
    {
        double c = cos(statePost.at<double>(2) + delta(1) / 2), s = sin(statePost.at<double>(2) + delta(1) / 2);
        transitionMatrix = (cv::Mat_<double>(3, 3) <<
            1, 0, delta(0) * -s,
            0, 1, delta(0) *  c,
            0, 0, 1);
        cv::Mat W = (cv::Mat_<double>(3, 2) <<
            c, -delta(0) * s,
            s,  delta(0) * c,
            0, 1);
        processNoiseCov = W * m_ctrl_noise * W.t();
        predict();
        statePre.at<double>(0) += delta(0) * c;
        statePre.at<double>(1) += delta(0) * s;
        statePre.at<double>(2) += delta(1);
        statePre.copyTo(statePost);
        errorCovPre.copyTo(errorCovPost);
        return true;
    }

    virtual bool applyGPS(const cv::Vec2d& gps)
    {
        correct(cv::Mat(gps));
        // No extra routine because its observation model is linear
        statePost.copyTo(statePre);
        errorCovPost.copyTo(errorCovPre);
        return true;
    }

    virtual bool setControlNoise(const cv::Vec2d& noise)
    {
        m_ctrl_noise = 0;
        m_ctrl_noise.at<double>(0, 0) = noise(0) * noise(0);
        m_ctrl_noise.at<double>(1, 1) = noise(1) * noise(1);
        return true;
    }

    virtual bool setObserveNoise(const cv::Vec2d& noise)
    {
        measurementNoiseCov = 0;
        measurementNoiseCov.at<double>(0, 0) = noise(0) * noise(0);
        measurementNoiseCov.at<double>(1, 1) = noise(1) * noise(1);
        return true;
    }

protected:
    cv::Mat m_ctrl_noise;
};

int testPoseEstimator(PoseEstimator* estimator, double odo_noise = 0.01, double gps_noise = 0.3, double interval = 0.1, double velocity = 1)
{
    if (estimator == nullptr) return -1;

    cv::RNG rng;
    if (!estimator->setControlNoise({ odo_noise, odo_noise })) return -1;
    if (!estimator->setObserveNoise({ gps_noise, gps_noise })) return -1;
    cv::Vec2d odo(velocity * interval, 0);
    printf("| Time [sec] | GPS Data [m] | Est. Position [m] | Est. Orientation [deg] |\n");
    printf("| ---------- | ------------ | ----------------- | ---------------------- |\n");
    for (double t = interval; t < 10; t += interval)
    {
        // Apply odometry displacement
        if (!estimator->applyOdometry(odo)) return -1;

        // Apply noisy GPS position
        cv::Vec2d gps(velocity * t, 1);
        gps(0) += rng.gaussian(gps_noise);
        gps(1) += rng.gaussian(gps_noise);
        if (!estimator->applyGPS(gps)) return -1;

        // Print the current pose
        cv::Vec3d pose = estimator->getPose();
        printf("| %.1f | %.3f, %.3f | %.3f, %.3f | %.0f |\n", t, gps(0), gps(1), pose(0), pose(1), cx::cvtRad2Deg(pose(2)));
    }
    return 0;
}

int main()
{
    printf("\n### Test PoseEstimatorCx with cx::EKF\n");
    PoseEstimatorCx estimator_cx;
    if (testPoseEstimator(&estimator_cx) < 0) return -1;

    printf("\n### Test PoseEstimatorCv with cv::KalmanFilter\n");
    PoseEstimatorCv estimator_cv;
    if (testPoseEstimator(&estimator_cv) < 0) return -1;

    return 0;
}
