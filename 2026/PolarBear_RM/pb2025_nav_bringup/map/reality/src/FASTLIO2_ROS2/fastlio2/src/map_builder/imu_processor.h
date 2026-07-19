#pragma once
#include "ieskf.h"
#include "commons.h"
#include <functional>

class IMUProcessor
{
public:
    IMUProcessor(Config &config, std::shared_ptr<IESKF> kf);

    bool initialize(SyncPackage &package);

    void undistort(SyncPackage &package);

    typedef std::function<void(double, const M3D&, const V3D&, const V3D&)> HighFreqOdomCallback;
    HighFreqOdomCallback m_odom_callback = nullptr;

    void set_odom_callback(HighFreqOdomCallback cb) { m_odom_callback = cb; }
    
private:
    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    double m_last_propagate_end_time;
    Vec<IMUData> m_imu_cache;
    Vec<Pose> m_poses_cache;
    V3D m_last_acc;
    V3D m_last_gyro;
    M12D m_Q;
    IMUData m_last_imu;
};
