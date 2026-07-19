#include "imu_processor.h"

IMUProcessor::IMUProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    m_Q.setIdentity();
    m_Q.block<3, 3>(0, 0) = M3D::Identity() * m_config.ng;
    m_Q.block<3, 3>(3, 3) = M3D::Identity() * m_config.na;
    m_Q.block<3, 3>(6, 6) = M3D::Identity() * m_config.nbg;
    m_Q.block<3, 3>(9, 9) = M3D::Identity() * m_config.nba;
    m_last_acc.setZero();
    m_last_gyro.setZero();
    m_imu_cache.clear();
    m_poses_cache.clear();
}

bool IMUProcessor::initialize(SyncPackage &package)
{
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());
    if (m_imu_cache.size() < static_cast<size_t>(m_config.imu_init_num))
        return false;
    V3D acc_mean = V3D::Zero();
    V3D gyro_mean = V3D::Zero();
    for (const auto &imu : m_imu_cache)
    {
        acc_mean += imu.acc;
        gyro_mean += imu.gyro;
    }
    acc_mean /= static_cast<double>(m_imu_cache.size());
    gyro_mean /= static_cast<double>(m_imu_cache.size());
    m_kf->x().r_il = m_config.r_il;
    m_kf->x().t_il = m_config.t_il;
    m_kf->x().bg = gyro_mean;
    if (m_config.gravity_align)
    {
        m_kf->x().r_wi = (Eigen::Quaterniond::FromTwoVectors((-acc_mean).normalized(), V3D(0.0, 0.0, -1.0)).matrix());
        m_kf->x().initGravityDir(V3D(0, 0, -1.0));
    }
    else
        m_kf->x().initGravityDir(-acc_mean);
    m_kf->P().setIdentity();
    m_kf->P().block<3, 3>(6, 6) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(9, 9) = M3D::Identity() * 0.00001;
    m_kf->P().block<3, 3>(15, 15) = M3D::Identity() * 0.0001;
    m_kf->P().block<3, 3>(18, 18) = M3D::Identity() * 0.0001;

    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = package.cloud_end_time;
    return true;
}

void IMUProcessor::undistort(SyncPackage &package)
{

    m_imu_cache.clear();
    m_imu_cache.push_back(m_last_imu);
    m_imu_cache.insert(m_imu_cache.end(), package.imus.begin(), package.imus.end());

    // const double imu_time_begin = m_imu_cache.front().time;
    const double imu_time_end = m_imu_cache.back().time;

    const double cloud_time_begin = package.cloud_start_time;
    const double propagate_time_end = package.cloud_end_time;

    m_poses_cache.clear();
    m_poses_cache.emplace_back(0.0, m_last_acc, m_last_gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi);

    V3D acc_val, gyro_val;
    double dt = 0.0;
    Input inp;
    inp.acc = m_imu_cache.back().acc;
    inp.gyro = m_imu_cache.back().gyro;
   // 找到 undistort 函数中的对应循环，修改如下：
    for (auto it_imu = m_imu_cache.begin(); it_imu < (m_imu_cache.end() - 1); it_imu++)
{
    // 修改点 1：删除 auto head = it_imu; 和 auto tail = it_imu + 1; 
    // 直接使用指针引用的方式
    IMUData &head_imu = *it_imu;
    IMUData &tail_imu = *(it_imu + 1);

    if (tail_imu.time < m_last_propagate_end_time) continue;

    V3D angvel_avl = 0.5 * (head_imu.gyro + tail_imu.gyro);
    V3D acc_avl = 0.5 * (head_imu.acc + tail_imu.acc);

    double dt;
    if (head_imu.time < m_last_propagate_end_time)
        dt = tail_imu.time - m_last_propagate_end_time;
    else
        dt = tail_imu.time - head_imu.time;

    Input inp;
    inp.acc = acc_avl;
    inp.gyro = angvel_avl;
    
    m_kf->predict(inp, dt, m_Q);

    // --- 修改点 2：高频发布回调，确保字段名为 .time 而不是 .timestamp ---
    if (m_odom_callback) {
        m_odom_callback(tail_imu.time, m_kf->x().r_wi, m_kf->x().t_wi, m_kf->x().v);
    }

    // --- 修改点 3：Pose 构造函数，确保字段匹配 ---
    m_poses_cache.push_back(Pose(tail_imu.time, tail_imu.acc, tail_imu.gyro, m_kf->x().v, m_kf->x().t_wi, m_kf->x().r_wi));
	}

    dt = propagate_time_end - imu_time_end;
    m_kf->predict(inp, dt, m_Q);
    m_last_imu = m_imu_cache.back();
    m_last_propagate_end_time = propagate_time_end;

    M3D cur_r_wi = m_kf->x().r_wi;
    V3D cur_t_wi = m_kf->x().t_wi;
    M3D cur_r_il = m_kf->x().r_il;
    V3D cur_t_il = m_kf->x().t_il;
    auto it_pcl = package.cloud->points.end() - 1;

    for (auto it_kp = m_poses_cache.end() - 1; it_kp != m_poses_cache.begin(); it_kp--)
    {
        auto head = it_kp - 1;
        auto tail = it_kp;

        M3D imu_r_wi = head->rot;
        V3D imu_t_wi = head->trans;
        V3D imu_vel = head->vel;
        V3D imu_acc = tail->acc;
        V3D imu_gyro = tail->gyro;

        for (; it_pcl->curvature / double(1000) > head->offset; it_pcl--)
        {
            dt = it_pcl->curvature / double(1000) - head->offset;
            V3D point(it_pcl->x, it_pcl->y, it_pcl->z);
            M3D point_rot = imu_r_wi * Sophus::SO3d::exp(imu_gyro * dt).matrix();
            V3D point_pos = imu_t_wi + imu_vel * dt + 0.5 * imu_acc * dt * dt;
            V3D p_compensate = cur_r_il.transpose() * (cur_r_wi.transpose() * (point_rot * (cur_r_il * point + cur_t_il) + point_pos - cur_t_wi) - cur_t_il);
            it_pcl->x = p_compensate(0);
            it_pcl->y = p_compensate(1);
            it_pcl->z = p_compensate(2);
            if (it_pcl == package.cloud->points.begin())
                break;
        }
    }
}
