#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/pcl_macros.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>

#include <cmath>
#include <ctime>
#include <array>
#include <string>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <queue>
#include <assert.h>

#include <Eigen/Dense>

#include "utils/timer.h"
#include "utils/math_tools.h"
#include "my_utility.h"
#include "lili_om_rot/cloud_info.h"

#include <vector>
#include <string>
using namespace std;
#define PI 3.1415926535

enum class SensorType
{
    VELODYNE,
    OUSTER,
    LIVOX
};

struct smoothness_t
{
    float value;
    size_t ind;
};

struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

using PointXYZIRT = VelodynePointXYZIRT;

class FeatureExtract
{
private:
    int downsampleRate = 2;
    double ds_v = 0.6;

    ros::NodeHandle nh;

    ros::Subscriber sub_Lidar_cloud;
    ros::Subscriber sub_imu;

    ros::Publisher pub_surf;
    ros::Publisher pub_edge;
    ros::Publisher pub_cutted_cloud;

    int pre_num = 0;

    pcl::PointCloud<PointType> lidar_cloud_in;
    std_msgs::Header cloud_header;

    vector<sensor_msgs::ImuConstPtr> imu_buf;
    int idx_imu = 0;
    double current_time_imu = -1;

    Eigen::Vector3d gyr_0;
    Eigen::Quaterniond qIMU = Eigen::Quaterniond::Identity();
    Eigen::Vector3d rIMU = Eigen::Vector3d::Zero();
    bool first_imu = false;
    string imu_topic;

    std::deque<sensor_msgs::PointCloud2> cloud_queue;
    sensor_msgs::PointCloud2 current_cloud_msg;
    double time_scan_next;
    double dt_ = 0;

    int N_SCANS = 64;

    double qlb0, qlb1, qlb2, qlb3;
    Eigen::Quaterniond q_lb;
    string frame_id = "lili_om_rot";
    string lidar_topic = "/velodyne_points";

    string sensorStr;
    SensorType sensor;
    int Horizon_SCAN;
    float lidarMinRange;
    float lidarMaxRange;

    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<OusterPointXYZIRT>::Ptr tmpOusterCloudIn;
    pcl::PointCloud<PointType>::Ptr inputCloud;
    pcl::PointCloud<PointType>::Ptr fullCloud;
    pcl::PointCloud<PointType>::Ptr extractedCloud;

    pcl::PointCloud<PointType>::Ptr cornerCloud;
    pcl::PointCloud<PointType>::Ptr surfaceCloud;

    pcl::VoxelGrid<PointType> downSizeFilter;
    float odometrySurfLeafSize;

    lili_om_rot::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    cv::Mat rangeMat;

    std::vector<smoothness_t> cloudSmoothness;
    float *cloudCurvature;
    int *cloudNeighborPicked;
    int *cloudLabel;

    double runtime = 0;

public:
    FeatureExtract() : nh("~")
    {
        if (!getParameter("/preprocessing/sensor", sensorStr))
        {
            ROS_WARN("lidar sensor type not set,error ...");
            return;
        }
        else
        {
            if (sensorStr == "velodyne")
                sensor = SensorType::VELODYNE;
            else if (sensorStr == "ouster")
                sensor = SensorType::OUSTER;
            else if (sensorStr == "livox")
                sensor = SensorType::LIVOX;
            else
            {
                ROS_ERROR_STREAM(
                    "Invalid sensor type (must be either 'velodyne', 'ouster' or 'livox'): " << sensorStr);
                ros::shutdown();
            }
            std::cout << "sensor type:" << sensorStr << std::endl;
        }

        if (!getParameter("/preprocessing/lidar_topic", lidar_topic))
        {
            ROS_WARN("lidar_topic not set, use default value: /velodyne_points");
            lidar_topic = "/velodyne_points";
        }
        std::cout << "lidar_topic: " << lidar_topic << std::endl;

        if (!getParameter("/preprocessing/line_num", N_SCANS))
        {
            ROS_WARN("line_num not set, use default value: 64");
            N_SCANS = 64;
        }

        if (!getParameter("/preprocessing/Horizon_SCAN", Horizon_SCAN))
        {
            ROS_WARN("Horizon_SCAN not set");
        }
        if (!getParameter("/preprocessing/lidarMinRange", lidarMinRange))
        {
            ROS_WARN("lidarMinRange not set");
        }
        if (!getParameter("/preprocessing/lidarMaxRange", lidarMaxRange))
        {
            ROS_WARN("lidarMaxRange not set");
        }

        if (!getParameter("/preprocessing/edgeThreshold", edgeThreshold))
        {
            ROS_WARN("edgeThreshold not set");
        }
        if (!getParameter("/preprocessing/surfThreshold", surfThreshold))
        {
            ROS_WARN("surfThreshold not set");
        }
        if (!getParameter("/preprocessing/edgeFeatureMinValidNum", edgeFeatureMinValidNum))
        {
            ROS_WARN("edgeFeatureMinValidNum not set");
        }
        if (!getParameter("/preprocessing/surfFeatureMinValidNum", surfFeatureMinValidNum))
        {
            ROS_WARN("surfFeatureMinValidNum not set");
        }
        if (!getParameter("/preprocessing/odometrySurfLeafSize", odometrySurfLeafSize))
        {
            ROS_WARN("odometrySurfLeafSize not set");
        }

        if (!getParameter("/preprocessing/ds_rate", downsampleRate))
        {
            ROS_WARN("ds_rate not set, use default value: 1");
            downsampleRate = 1;
        }

        if (!getParameter("/common/frame_id", frame_id))
        {
            ROS_WARN("frame_id not set, use default value: lili_odom");
            frame_id = "lili_odom";
        }

        if (!getParameter("/backend_fusion/imu_topic", imu_topic))
        {
            ROS_WARN("imu_topic not set, use default value: /imu/data");
            imu_topic = "/imu/data";
        }
        std::cout << "imu_topic: " << imu_topic << std::endl;

        // extrinsic parameters
        if (!getParameter("/backend_fusion/ql2b_w", qlb0))
        {
            ROS_WARN("ql2b_w not set, use default value: 1");
            qlb0 = 1;
        }

        if (!getParameter("/backend_fusion/ql2b_x", qlb1))
        {
            ROS_WARN("ql2b_x not set, use default value: 0");
            qlb1 = 0;
        }

        if (!getParameter("/backend_fusion/ql2b_y", qlb2))
        {
            ROS_WARN("ql2b_y not set, use default value: 0");
            qlb2 = 0;
        }

        if (!getParameter("/backend_fusion/ql2b_z", qlb3))
        {
            ROS_WARN("ql2b_z not set, use default value: 0");
            qlb3 = 0;
        }

        q_lb = Eigen::Quaterniond(qlb0, qlb1, qlb2, qlb3);

        sub_Lidar_cloud = nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, &FeatureExtract::cloudHandler, this);
        sub_imu = nh.subscribe<sensor_msgs::Imu>(imu_topic, 200, &FeatureExtract::imuHandler, this);

        pub_surf = nh.advertise<sensor_msgs::PointCloud2>("/surf_features", 100);
        pub_edge = nh.advertise<sensor_msgs::PointCloud2>("/edge_features", 100);
        pub_cutted_cloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar_cloud_cutted", 100);

        allocateMemory();
        resetParameters();
    }

    ~FeatureExtract() {}

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        tmpOusterCloudIn.reset(new pcl::PointCloud<OusterPointXYZIRT>());
        inputCloud.reset(new pcl::PointCloud<PointType>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        fullCloud->points.resize(N_SCANS * Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCANS, 0);
        cloudInfo.endRingIndex.assign(N_SCANS, 0);

        cloudInfo.pointColInd.assign(N_SCANS * Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCANS * Horizon_SCAN, 0);

        cloudSmoothness.resize(N_SCANS * Horizon_SCAN);

        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        cloudCurvature = new float[N_SCANS * Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCANS * Horizon_SCAN];
        cloudLabel = new int[N_SCANS * Horizon_SCAN];

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        inputCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCANS, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    }

    template <typename PointT>
    double getDepth(PointT pt)
    {
        return sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
    }

    PointType undistortion(PointType pt, double ratio_i, const Eigen::Quaterniond quat)
    {

        if (ratio_i >= 1.0)
        {
            ratio_i = 1.0;
        }

        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Quaterniond q_si = q0.slerp(ratio_i, qIMU);

        Eigen::Vector3d pt_i(pt.x, pt.y, pt.z);

        q_si = q_lb * q_si * q_lb.inverse();
        Eigen::Vector3d pt_s = q_si * pt_i;

        PointType p_out;
        p_out.x = pt_s.x();
        p_out.y = pt_s.y();
        p_out.z = pt_s.z();
        p_out.intensity = pt.intensity;
        return p_out;
    }

    void solveRotation(double dt, Eigen::Vector3d angular_velocity)
    {
        Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity);
        qIMU *= deltaQ(un_gyr * dt);
        gyr_0 = angular_velocity;
    }

    void processIMU(double t_cur)
    {
        double rx = 0, ry = 0, rz = 0;
        int i = idx_imu;
        if (i >= imu_buf.size())
            i--;
        while (imu_buf[i]->header.stamp.toSec() < t_cur)
        {

            double t = imu_buf[i]->header.stamp.toSec();
            if (current_time_imu < 0)
                current_time_imu = t;
            double dt = t - current_time_imu;
            current_time_imu = imu_buf[i]->header.stamp.toSec();

            rx = imu_buf[i]->angular_velocity.x;
            ry = imu_buf[i]->angular_velocity.y;
            rz = imu_buf[i]->angular_velocity.z;
            solveRotation(dt, Eigen::Vector3d(rx, ry, rz));
            i++;
            if (i >= imu_buf.size())
                break;
        }

        if (i < imu_buf.size())
        {
            double dt1 = t_cur - current_time_imu;
            double dt2 = imu_buf[i]->header.stamp.toSec() - t_cur;

            double w1 = dt2 / (dt1 + dt2);
            double w2 = dt1 / (dt1 + dt2);

            rx = w1 * rx + w2 * imu_buf[i]->angular_velocity.x;
            ry = w1 * ry + w2 * imu_buf[i]->angular_velocity.y;
            rz = w1 * rz + w2 * imu_buf[i]->angular_velocity.z;
            solveRotation(dt1, Eigen::Vector3d(rx, ry, rz));
        }
        current_time_imu = t_cur;
        idx_imu = i;
    }

    void imuHandler(const sensor_msgs::ImuConstPtr &ImuIn)
    {
        imu_buf.push_back(ImuIn);

        if (imu_buf.size() > 600)
            imu_buf[imu_buf.size() - 601] = nullptr;

        if (current_time_imu < 0)
            current_time_imu = ImuIn->header.stamp.toSec();

        if (!first_imu)
        {
            first_imu = true;
            double rx = 0, ry = 0, rz = 0;
            rx = ImuIn->angular_velocity.x;
            ry = ImuIn->angular_velocity.y;
            rz = ImuIn->angular_velocity.z;
            Eigen::Vector3d angular_velocity(rx, ry, rz);
            gyr_0 = angular_velocity;
        }
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        cloud_queue.push_back(*laserCloudMsg);
        if (cloud_queue.size() <= 2)
            return false;

        //  convert cloud
        current_cloud_msg = std::move(cloud_queue.front());
        cloud_queue.pop_front();
        cloud_header = current_cloud_msg.header;
        cloud_header.frame_id = frame_id;
        time_scan_next = cloud_queue.front().header.stamp.toSec();

        if (sensor == SensorType::VELODYNE)
        {
            pcl::moveFromROSMsg(current_cloud_msg, *laserCloudIn);
        }
        else if (sensor == SensorType::OUSTER)
        {
            // Convert to Velodyne format
            pcl::fromROSMsg(current_cloud_msg, *tmpOusterCloudIn);
            // pcl::moveFromROSMsg(current_cloud_msg, *tmpOusterCloudIn);
            laserCloudIn->points.resize(tmpOusterCloudIn->size());
            laserCloudIn->is_dense = tmpOusterCloudIn->is_dense;
            for (size_t i = 0; i < tmpOusterCloudIn->size(); i++)
            {
                auto &src = tmpOusterCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];
                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.t * 1e-9f;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }
        dt_ = time_scan_next - current_cloud_msg.header.stamp.toSec(); //  per scan
        // std::cout << "scan period: " << dt_ << std::endl;

        // check dense flag
        if (inputCloud->is_dense == false)
        {
            ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
            ros::shutdown();
        }

        // check ring channel
        static int ringFlag = 0;
        if (ringFlag == 0)
        {
            ringFlag = -1;
            for (int i = 0; i < (int)current_cloud_msg.fields.size(); ++i)
            {
                if (current_cloud_msg.fields[i].name == "ring")
                {
                    ringFlag = 1;
                    break;
                }
            }
            if (ringFlag == -1)
            {
                ROS_ERROR("Point cloud ring channel not available, please configure your point cloud data!");
                ros::shutdown();
            }
        }

        return true;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCANS)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

            static float ang_res_x = 360.0 / float(Horizon_SCAN);
            int columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            int index = columnIdn + rowIdn * Horizon_SCAN;
            thisPoint = undistortion(thisPoint, laserCloudIn->points[i].time / dt_, qIMU);
            fullCloud->points[index] = thisPoint;
        }
    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCANS; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i, j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i, j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i * Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count - 1 - 5;
        }
    }

    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] + cloudInfo.pointRange[i + 5];

            cloudCurvature[i] = diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;

            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;
            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];
            cloudSmoothness[i].ind = i;
        }
    }

    void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i + 1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

            if (columnDiff < 10)
            {
                // 10 pixel diff in range image
                if (depth1 - depth2 > 0.3)
                {
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }
                else if (depth2 - depth1 > 0.3)
                {
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    void extractFeatures()
    {
        cornerCloud->clear();
        surfaceCloud->clear();

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < N_SCANS; i++)
        {

            surfaceCloudScan->clear();

            for (int j = 0; j < 6; j++)
            {

                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                int largestPickedNum = 0;
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        largestPickedNum++;
                        if (largestPickedNum <= 20)
                        {
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        }
                        else
                        {
                            break;
                        }

                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++)
                        {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0)
                    {
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        // std::cout << "surf: " << surfaceCloud->size() << std::endl;
        freeCloudInfoMemory();
        // std::cout << "edge: " << cornerCloud->size() << std::endl;
        sensor_msgs::PointCloud2 laserCloudOutMsg;
        pcl::toROSMsg(*extractedCloud, laserCloudOutMsg);
        laserCloudOutMsg.header.stamp = current_cloud_msg.header.stamp;
        laserCloudOutMsg.header.frame_id = frame_id;
        pub_cutted_cloud.publish(laserCloudOutMsg);

        sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;
        pcl::toROSMsg(*cornerCloud, cornerPointsLessSharpMsg);
        cornerPointsLessSharpMsg.header.stamp = current_cloud_msg.header.stamp;
        cornerPointsLessSharpMsg.header.frame_id = frame_id;
        pub_edge.publish(cornerPointsLessSharpMsg);

        sensor_msgs::PointCloud2 surfPointsLessFlat2;
        pcl::toROSMsg(*surfaceCloud, surfPointsLessFlat2);
        surfPointsLessFlat2.header.stamp = current_cloud_msg.header.stamp;
        surfPointsLessFlat2.header.frame_id = frame_id;
        pub_surf.publish(surfPointsLessFlat2);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
    {
        // cache point cloud
        if (!cachePointCloud(laserCloudMsg))
            return;

        int tmpIdx = 0;
        if (idx_imu > 0)
            tmpIdx = idx_imu - 1;
        if (imu_buf.empty() || imu_buf[tmpIdx]->header.stamp.toSec() > time_scan_next)
        {
            ROS_WARN("Waiting for IMU data ...");
            return;
        }

        if (first_imu)
            processIMU(time_scan_next);
        if (isnan(qIMU.w()) || isnan(qIMU.x()) || isnan(qIMU.y()) || isnan(qIMU.z()))
        {
            qIMU = Eigen::Quaterniond::Identity();
        }

        projectPointCloud();

        cloudExtraction();

        calculateSmoothness();

        markOccludedPoints();

        extractFeatures();

        publishFeatureCloud();

        qIMU = Eigen::Quaterniond::Identity();
        rIMU = Eigen::Vector3d::Zero();
        resetParameters();
        // cout<<"pre_num: "<<++pre_num<<endl;
        // cout<<"Preprocessing average run time: "<<runtime / pre_num<<endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lili_om_rot");
    FeatureExtract Pre;
    ROS_INFO("\033[1;32m---->\033[0m FeatureExtract Started.");

    ros::spin();
    return 0;
}
