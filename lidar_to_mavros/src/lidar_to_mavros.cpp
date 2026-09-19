#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// 雷达-飞控安装偏差补偿: 将雷达位置转换为飞控位置
struct LeverArmCompensator
{
    bool   enabled;
    double offset_x;   // 机体系: 向前为+
    double offset_y;   // 机体系: 向左为+
    double offset_z;   // 机体系: 向上为+

    LeverArmCompensator(): enabled(true), offset_x(0.0), offset_y(0.0), offset_z(0.0) {}

    void load(const ros::NodeHandle &nh_private)
    {
        nh_private.param("lidar_offset_en", enabled, true);
        nh_private.param("lidar_offset_x",  offset_x,  0.0);
        nh_private.param("lidar_offset_y",  offset_y,  0.0);
        nh_private.param("lidar_offset_z",  offset_z,  0.0);
    }

    void print() const
    {
        ROS_INFO("[LeverArm] compensation=%s, body offset=(%.3f, %.3f, %.3f)m",
                 enabled ? "ON" : "OFF", offset_x, offset_y, offset_z);
    }

    // 将雷达位置转换为飞控位置 (绝对补偿, 无起点对齐)
    // P_飞控 = P_雷达 - R × offset
    void apply(geometry_msgs::PoseStamped &pose) const
    {
        if (!enabled) return;

        tf2::Quaternion q;
        tf2::fromMsg(pose.pose.orientation, q);
        tf2::Vector3 body_offset(offset_x, offset_y, offset_z);
        tf2::Vector3 world_offset = tf2::quatRotate(q, body_offset);

        pose.pose.position.x -= world_offset.x();
        pose.pose.position.y -= world_offset.y();
        pose.pose.position.z -= world_offset.z();
    }
};

// 输出层位置EKF: 三轴独立的「位置+速度」匀速(CV)模型滤波
// 里程计(约10Hz)到达时做更新, 发布循环外推到当前时刻, 姿态不滤直接透传
// 收益: 10Hz阶梯位置 -> 高频连续输出; 马氏门限剔除里程计野值; 短时丢帧匀速外推
struct PoseEKF
{
    bool   enabled;
    double sigma_a;      // 过程噪声: 加速度白噪声 (m/s^2), 调小更平滑、调大更跟测
    double sigma_r;      // 观测噪声: 位置测量std (m)
    double gate_thresh;  // 新息马氏距离平方门限 (3自由度chi2: 95%=7.81, 99%=11.35)
    int    max_rejects;  // 连续拒收上限, 超过后重置到观测(应对真实大位移)
    double extrap_max;   // 无更新时最大外推时长 (s), 超出则保持在外推边界

    struct Axis { double p, v; double P00, P01, P10, P11; };
    Axis ax[3];

    bool   initialized;
    double last_update_t;  // 最近一次更新对应的里程计时间 (s)
    int    reject_count;

    PoseEKF(): enabled(true), sigma_a(2.0), sigma_r(0.03), gate_thresh(11.35),
               max_rejects(15), extrap_max(0.5), initialized(false),
               last_update_t(0.0), reject_count(0) {}

    void load(const ros::NodeHandle &nh_private)
    {
        nh_private.param("kf_en",          enabled,     true);
        nh_private.param("kf_sigma_a",     sigma_a,     2.0);
        nh_private.param("kf_sigma_r",     sigma_r,     0.03);
        nh_private.param("kf_gate_thresh", gate_thresh, 11.35);
        nh_private.param("kf_max_rejects", max_rejects, 15);
        nh_private.param("kf_extrap_max",  extrap_max,  0.5);
        ROS_INFO("[PoseEKF] enabled=%s, sigma_a=%.2f m/s^2, sigma_r=%.3f m, gate=%.1f, max_rejects=%d, extrap_max=%.2fs",
                 enabled ? "ON" : "OFF", sigma_a, sigma_r, gate_thresh, max_rejects, extrap_max);
    }

    void reset(const double z[3], double t)
    {
        for (int i = 0; i < 3; i++)
        {
            ax[i].p   = z[i];
            ax[i].v   = 0.0;
            ax[i].P00 = sigma_r * sigma_r;
            ax[i].P01 = ax[i].P10 = 0.0;
            ax[i].P11 = 25.0;   // 速度初始不确定度 (5 m/s)^2
        }
        initialized = true;
        last_update_t = t;
        reject_count = 0;
    }

    void predict(double dt)
    {
        if (dt <= 0.0) return;
        double dt2 = dt * dt;
        double q = sigma_a * sigma_a;
        for (int i = 0; i < 3; i++)
        {
            Axis &a = ax[i];
            a.p += a.v * dt;
            double P00n = a.P00 + dt * (a.P10 + a.P01) + dt2 * a.P11 + q * dt2 * dt2 / 4.0;
            double P01n = a.P01 + dt * a.P11 + q * dt2 * dt / 2.0;
            double P10n = a.P10 + dt * a.P11 + q * dt2 * dt / 2.0;
            double P11n = a.P11 + q * dt2;
            a.P00 = P00n; a.P01 = P01n; a.P10 = P10n; a.P11 = P11n;
        }
    }

    // 里程计位置到达: 预测到t -> 门限判据 -> 更新, 滤波位置回写
    void filterPosition(geometry_msgs::Point &pos, double t)
    {
        if (!enabled) return;

        double z[3] = {pos.x, pos.y, pos.z};
        if (std::isnan(z[0]) || std::isnan(z[1]) || std::isnan(z[2])) return;

        if (!initialized) { reset(z, t); writeBack(pos); return; }

        double dt = t - last_update_t;
        if (dt < 0.0) dt = 0.0;   // 时间回跳时不预测
        predict(dt);
        last_update_t = t;

        double r2 = sigma_r * sigma_r;
        double d2 = 0.0;
        for (int i = 0; i < 3; i++)
        {
            double S = ax[i].P00 + r2;
            double y = z[i] - ax[i].p;
            d2 += y * y / S;
        }

        if (d2 > gate_thresh)
        {
            if (++reject_count >= max_rejects)
            {
                // 持续性偏移: 判定为真实大位移, 重置跟随
                ROS_WARN("[PoseEKF] %d consecutive rejects (d2=%.1f), reset to measurement",
                         reject_count, d2);
                reset(z, t);
            }
            else if (reject_count == 1)
            {
                // 瞬时野值: 拒收, 输出按预测值平滑过渡
                ROS_WARN("[PoseEKF] outlier rejected (d2=%.1f > %.1f), coasting on prediction",
                         d2, gate_thresh);
            }
        }
        else
        {
            reject_count = 0;
            for (int i = 0; i < 3; i++)
            {
                Axis &a = ax[i];
                double S  = a.P00 + r2;
                double K0 = a.P00 / S;
                double K1 = a.P10 / S;
                double y  = z[i] - a.p;
                a.p += K0 * y;
                a.v += K1 * y;
                double P00n = (1.0 - K0) * a.P00;
                double P01n = (1.0 - K0) * a.P01;
                double P10n = a.P10 - K1 * a.P00;
                double P11n = a.P11 - K1 * a.P01;
                a.P00 = P00n; a.P01 = P01n; a.P10 = P10n; a.P11 = P11n;
            }
        }
        writeBack(pos);
    }

    void writeBack(geometry_msgs::Point &pos) const
    {
        pos.x = ax[0].p; pos.y = ax[1].p; pos.z = ax[2].p;
    }

    // 发布用: 从最近更新时刻外推到t_out, 不改变内部状态
    void extrapolateTo(double t_out, geometry_msgs::Point &pos, double &stamp_out) const
    {
        double dt = t_out - last_update_t;
        if (dt < 0.0)        dt = 0.0;
        if (dt > extrap_max) dt = extrap_max;
        pos.x = ax[0].p + ax[0].v * dt;
        pos.y = ax[1].p + ax[1].v * dt;
        pos.z = ax[2].p + ax[2].v * dt;
        stamp_out = last_update_t + dt;
    }
};

class vision_pose
{
public:
    vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
    double pi;

    struct attitude { double pitch; double roll; double yaw; };

    attitude estimatedAttitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped estimatedPose;

    bool estimatedOdomRec_flag;

    ros::Rate *rate;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber px4Pose_sub;
    ros::Publisher vision_pose_pub;
    ros::Subscriber odom_sub;

    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void start();

    // ====== 零漂校准 ======
    struct ZeroDriftCalibrator
    {
        bool enabled;
        double calib_time;
        double motion_thresh;

        enum State { WAITING, CALIBRATING, DONE, FAILED };
        State state;

        ros::Time start_time;
        double sum_x, sum_y, sum_z;
        int count;
        double offset_x, offset_y, offset_z;
        double first_x, first_y, first_z;

        ZeroDriftCalibrator()
            : enabled(true), calib_time(2.0), motion_thresh(0.05),
              state(WAITING), count(0),
              sum_x(0), sum_y(0), sum_z(0),
              offset_x(0), offset_y(0), offset_z(0),
              first_x(0), first_y(0), first_z(0) {}

        void load(const ros::NodeHandle &nh_private)
        {
            nh_private.param("zero_drift_calib_en", enabled, true);
            nh_private.param("zero_drift_calib_time", calib_time, 2.0);
            nh_private.param("zero_drift_motion_thresh", motion_thresh, 0.05);
            ROS_INFO("[ZeroDrift] calibration=%s, time=%.1fs, motion_thresh=%.2fcm",
                     enabled ? "ON" : "OFF", calib_time, motion_thresh * 100);
        }

        void process(geometry_msgs::PoseStamped &pose)
        {
            if (!enabled) return;

            double x = pose.pose.position.x;
            double y = pose.pose.position.y;
            double z = pose.pose.position.z;

            switch (state)
            {
            case WAITING:
                start_time = ros::Time::now();
                first_x = x; first_y = y; first_z = z;
                state = CALIBRATING;
                ROS_INFO("[ZeroDrift] Started, collecting %.1fs of static data...", calib_time);
                break;

            case CALIBRATING:
            {
                sum_x += x; sum_y += y; sum_z += z;
                count++;

                double dx = x - first_x, dy = y - first_y, dz = z - first_z;
                double motion = sqrt(dx*dx + dy*dy + dz*dz);

                if (motion > motion_thresh)
                {
                    state = FAILED;
                    ROS_ERROR("\033[K\033[31m[ZeroDrift] FAILED! Motion detected (%.1fcm > %.1fcm). "
                              "Outputting raw data. RESTART if static.\033[0m",
                              motion*100, motion_thresh*100);
                }
                else if ((ros::Time::now() - start_time).toSec() >= calib_time)
                {
                    offset_x = sum_x / count;
                    offset_y = sum_y / count;
                    offset_z = sum_z / count;
                    state = DONE;
                    ROS_INFO("\033[K\033[32m[ZeroDrift] Done! offset=(%.4f, %.4f, %.4f)m, samples=%d\033[0m",
                             offset_x, offset_y, offset_z, count);
                }
                break;
            }

            case DONE:
                pose.pose.position.x -= offset_x;
                pose.pose.position.y -= offset_y;
                pose.pose.position.z -= offset_z;
                break;

            case FAILED:
                break;
            }
        }

        bool isCalibrating() const { return enabled && state == CALIBRATING; }
        bool isFailed() const { return enabled && state == FAILED; }
        double elapsed() const { return (ros::Time::now() - start_time).toSec(); }
    };

    ZeroDriftCalibrator zero_drift;
    LeverArmCompensator lever_arm;
    PoseEKF pose_kf;
};

vision_pose::vision_pose(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_) : nh(nh_), nh_private(nh_private_)
{
    pi = 3.1415926;

    // 发布频率: 上限100Hz(PX4 EKF2对外部视觉的有效带宽约30-50Hz, 更高只增加流量无收益)
    double publish_rate_hz = 100.0;
    nh_private.param("publish_rate", publish_rate_hz, 100.0);
    rate = new ros::Rate(publish_rate_hz);
    ROS_INFO("[lidar_to_mavros] publish rate = %.1f Hz", publish_rate_hz);

    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &vision_pose::px4Pose_cb, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 2, &vision_pose::estimator_odom_cb, this);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    estimatedOdomRec_flag = false;
    estimatedAttitude.pitch = estimatedAttitude.roll = estimatedAttitude.yaw = 0;
    px4Attitude.pitch = px4Attitude.roll = px4Attitude.yaw = 0;

    zero_drift.load(nh_private);
    lever_arm.load(nh_private);
    pose_kf.load(nh_private);
}

void vision_pose::estimator_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    estimatedPose.pose = msg->pose.pose;
    estimatedPose.header.stamp = msg->header.stamp;

    // 四元数转 RPY（用于调试显示）
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    estimatedAttitude.pitch = pitch * 180 / pi;
    estimatedAttitude.roll = roll * 180 / pi;
    estimatedAttitude.yaw = yaw * 180 / pi;

    // 位置EKF更新(姿态透传), 位于零漂校准之前
    pose_kf.filterPosition(estimatedPose.pose.position, msg->header.stamp.toSec());

    // 零漂校准处理
    zero_drift.process(estimatedPose);

    estimatedOdomRec_flag = true;
}

void vision_pose::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4Pose.pose = msg->pose;
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

void vision_pose::start()
{
    double last_print = 0.0;

    while (ros::ok())
    {
        double now_sec = ros::Time::now().toSec();

        // 控制台打印限频至2Hz, 避免高频发布循环下终端IO拖累CPU
        bool do_print = (now_sec - last_print) >= 0.5;
        if (do_print) last_print = now_sec;

        if (estimatedOdomRec_flag == false)
        {
            if (do_print)
                cout << "\033[K"
                     << "\033[31m visionPose no receive!!!  Waiting for pose\033[0m" << endl;
        }
        else
        {
            // 校准中不发布，显示进度
            if (zero_drift.isCalibrating())
            {
                if (do_print)
                {
                    printf("\033[K\033[33m [ZeroDrift] Calibrating... %.1f/%.1fs (keep STATIC)\033[0m\n",
                           zero_drift.elapsed(), zero_drift.calib_time);
                    printf("\033[1A");
                    fflush(stdout);
                }
            }
            else if (zero_drift.isFailed())
            {
                if (do_print)
                {
                    printf("\033[K\033[31m [ZeroDrift] FAILED! Outputting raw (uncorrected) data. Restart to retry.\033[0m\n");
                    fflush(stdout);
                }

                geometry_msgs::PoseStamped out_pose = estimatedPose;
                double stamp_sec = out_pose.header.stamp.toSec();
                if (pose_kf.enabled && pose_kf.initialized)
                    pose_kf.extrapolateTo(now_sec, out_pose.pose.position, stamp_sec);
                out_pose.header.stamp = ros::Time().fromSec(stamp_sec);

                lever_arm.apply(out_pose);
                vision_pose_pub.publish(out_pose);
            }
            else
            {
                // 正常运行: EKF外推到当前时刻 -> 杠臂补偿 -> 发布
                geometry_msgs::PoseStamped out_pose = estimatedPose;
                double stamp_sec = out_pose.header.stamp.toSec();
                if (pose_kf.enabled && pose_kf.initialized)
                    pose_kf.extrapolateTo(now_sec, out_pose.pose.position, stamp_sec);
                out_pose.header.stamp = ros::Time().fromSec(stamp_sec);

                lever_arm.apply(out_pose);
                vision_pose_pub.publish(out_pose);

                if (do_print)
                {
                    cout << "\033[K"
                         << "\033[32m estimate ok !\033[0m" << endl;
                    cout << "\033[K"
                         << "       Vision-Pose               Px4-Pose" << endl;
                    cout << setiosflags(ios::fixed) << setprecision(7)
                         << "\033[K"
                         << "x      " << out_pose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
                    cout << setiosflags(ios::fixed) << setprecision(7)
                         << "\033[K"
                         << "y      " << out_pose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
                    cout << setiosflags(ios::fixed) << setprecision(7)
                         << "\033[K"
                         << "z      " << out_pose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
                    cout << setiosflags(ios::fixed) << setprecision(7)
                         << "\033[K"
                         << "pitch  " << estimatedAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
                    cout << setiosflags(ios::fixed) << setprecision(7)
                         << "\033[K"
                         << "roll   " << estimatedAttitude.roll << "\t\t" << px4Attitude.roll << endl;
                    cout << setiosflags(ios::fixed) << setprecision(7)
                         << "\033[K"
                         << "yaw    " << estimatedAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
                    cout << "\033[9A" << endl;
                }
            }
        }
        ros::spinOnce();
        rate->sleep();
    }
    cout << "\033[9B" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_to_mavros");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");

    vision_pose vision(nh_, nh_private_);
    vision.start();

    return 0;
}

