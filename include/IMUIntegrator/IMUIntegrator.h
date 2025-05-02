#ifndef LIO_LIVOX_IMUINTEGRATOR_H
#define LIO_LIVOX_IMUINTEGRATOR_H

#include <Eigen/Eigen>
#include <sensor_msgs/msg/imu.hpp>

class IMUIntegrator {
public:
    IMUIntegrator();

    /** \brief constructor of IMUIntegrator
     * \param[in] vIMU: IMU messages need to be integrated
     */
    explicit IMUIntegrator(std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> vIMU);

    void Reset();

    /** \brief get delta quaternion after IMU integration
     */
    const Eigen::Quaterniond& GetDeltaQ() const;

    /** \brief get delta displacement after IMU integration
     */
    const Eigen::Vector3d& GetDeltaP() const;

    /** \brief get delta velocity after IMU integration
     */
    const Eigen::Vector3d& GetDeltaV() const;

    /** \brief get time span after IMU integration
     */
    const double& GetDeltaTime() const;

    /** \brief get linearized bias gyr
     */
    const Eigen::Vector3d& GetBiasGyr() const;

    /** \brief get linearized bias acc
     */
    const Eigen::Vector3d& GetBiasAcc() const;

    /** \brief get covariance matrix after IMU integration
     */
    const Eigen::Matrix<double, 15, 15>& GetCovariance();

    /** \brief get jacobian matrix after IMU integration
     */
    const Eigen::Matrix<double, 15, 15>& GetJacobian() const;

    /** \brief get average acceleration of IMU messages for initialization
     */
    Eigen::Vector3d GetAverageAcc();

    /** \brief push IMU message to the IMU buffer vimuMsg
     * \param[in] imu: the IMU message need to be pushed
     */
    void PushIMUMsg(const sensor_msgs::msg::Imu::ConstSharedPtr& imu);
    void PushIMUMsg(const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>& vimu);
    const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr>& GetIMUMsg() const;

    /** \brief only integrate gyro information of each IMU message stored in vimuMsg
     * \param[in] lastTime: the left time boundary of vimuMsg
     */
    void GyroIntegration(double lastTime);

    /** \brief pre-integration of IMU messages stored in vimuMsg
     */
    void PreIntegration(double lastTime, const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);

    /** \brief normal integration of IMU messages stored in vimuMsg
     */
    void Integration() {}

public:
    const double acc_n                    = 0.08;
    const double gyr_n                    = 0.004;
    const double acc_w                    = 2.0e-4;
    const double gyr_w                    = 2.0e-5;
    constexpr static const double lidar_m = 1.5e-3;
    constexpr static const double gnorm   = 9.805;

    enum JacobianOrder { O_P = 0, O_R = 3, O_V = 6, O_BG = 9, O_BA = 12 };

private:
    std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> vimuMsg;
    Eigen::Quaterniond dq;
    Eigen::Vector3d dp;
    Eigen::Vector3d dv;
    Eigen::Vector3d linearized_bg;
    Eigen::Vector3d linearized_ba;
    Eigen::Matrix<double, 15, 15> covariance;
    Eigen::Matrix<double, 15, 15> jacobian;
    Eigen::Matrix<double, 12, 12> noise;
    double dtime;
};

#endif // LIO_LIVOX_IMUINTEGRATOR_H
