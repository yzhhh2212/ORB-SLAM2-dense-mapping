#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <mutex>
#include <pcl/point_types.h>
#include "PointCloudMapping.h"
#include "KeyFrame.h"
#include "Frame.h"
#include <thread>
#include <pcl/visualization/pcl_visualizer.h>
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Converter.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <X11/Xlib.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>

namespace ORB_SLAM2
{
    PointCloudMapping::PointCloudMapping()
    {
        // mpGlobalCloud = pcl::PointCloud<pcl::PointXYZRGBA>::makeShared();
        // pcl::make_shared<pcl::PointXYZRGBA>( );
        // XInitThreads();
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGBA>);
        mpGlobalCloud = cloud1;
        mflag = 0;
        mkk = 0;
    }

    void PointCloudMapping::DisplayPointCloud()
    {

        // XInitThreads();

        // //pcl::visualization::CloudViewer viewer1("Cloud Viewer");
        // //viewer1.showCloud(mpGlobalCloud);

        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCViewer"));
        //  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //  // tmp = mpGlobalCloud;
        //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(mpGlobalCloud);
        //  viewer->addPointCloud<pcl::PointXYZRGBA>(mpGlobalCloud, rgb, "cloud");
        //  viewer->setBackgroundColor(0, 0, 0);
        //  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
        //  viewer->initCameraParameters();

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        KeyFrame *pKF;

        // pcl::visualization::CloudViewer viewer("Cloud Viewer");
        int lastN = 0;
        bool flag = 0;
        int updateN = 0;
        while (1)
        {

            // viewer->spinOnce(100);

            unique_lock<std::mutex> lock(mMutexPCKF);
            int N = mvpKF.size();
            if (N > lastN)
            {
                for (; lastN < N; lastN++)
                {
                    cout << "N ;" << N << endl;
                    cout << "lastN: " << lastN << endl;
                    // unique_lock<std::mutex> lock(mMutexPCKF);
                    {
                        pKF = mvpKF[lastN];
                    }
                    cloud = GenerateCloud(pKF);
                    *mpGlobalCloud += *cloud;
                    // viewer->updatePointCloud(mpGlobalCloud, "cloud");
                }

                // lastN = N ;
            }

            // viewer.showCloud(mpGlobalCloud);
            // std::this_thread::sleep_for(1000ms);
        }
    }

    void PointCloudMapping::UpdatePointCloud()
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp1(new pcl::PointCloud<pcl::PointXYZRGBA>);

        for (int i = 0; i < mvpKF.size(); i++)
        {
            KeyFrame *pKF = mvpKF[i];
            if (pKF->isBad())
                continue;
            else
            {
                cv::Mat Twc = pKF->GetPoseInverse();
                Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp2(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp3(new pcl::PointCloud<pcl::PointXYZRGBA>);
                tmp2 = pKF->GetPointCloud();
                pcl::transformPointCloud(*tmp2, *tmp3, T1.matrix());
                *tmp1 += *tmp3;
            }
        }
        mpGlobalCloud = tmp1;
    }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr PointCloudMapping::GenerateCloud(KeyFrame *pKF)
    {
        float z, x, y;
        cv::Mat Twc = pKF->GetPoseInverse();
        Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);
        cv::Mat Depth = pKF->mimDepthForPC;
        cv::Mat RGB = pKF->mimRGBForPC;
        pcl::PointXYZRGBA p;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);

        cout << "正在生成帧ID ：" << pKF->mnId << "的点云" << endl
             << endl;
        for (int v = 0; v < Depth.rows; v++)
        {
            for (int u = 0; u < Depth.cols; u++)
            {
                // 获取每个像素的深度值
                z = Depth.at<float>(v, u);
                if (z < 0.01 || z > 5)
                    continue;
                x = (u - pKF->cx) * z * pKF->invfx;
                y = (v - pKF->cy) * z * pKF->invfy;

                p.b = RGB.ptr<uchar>(v)[u * 3];
                p.g = RGB.ptr<uchar>(v)[u * 3 + 1];
                p.r = RGB.ptr<uchar>(v)[u * 3 + 2];
                p.x = x;
                p.y = y;
                p.z = z;
                cloud->push_back(p);
            }
        }

        pKF->SetPointCloud(cloud); // 帧坐标下的点云
        pcl::transformPointCloud(*cloud, *cloud2, T1.matrix());
        return cloud2;
    }

    void PointCloudMapping::InsertKeyFrame(KeyFrame *pKF)
    {

        // cv::Mat Depth = pKF->mimDepthForPC;
        // mimDepthForPC = pKF->mimDepthForPC;
        // mvimDepthForPC.push_back(Depth);
        mkk++;
        cout << "关键帧  : " << mkk << endl;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cv::Mat DepthOri = pKF->mimDepthForPC;
        mvimDepthOriForPC.push_back(DepthOri);

        unique_lock<std::mutex> lock(mMutexPCKF);
        {
            mvpKF.push_back(pKF);
        }
        // cloud = GenerateCloud(pKF);

        // unique_lock<std::mutex> lock(mMutexPC);
        // *mpGlobalCloud += *cloud;

        // float z, x, y;
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // pcl::PointXYZRGBA p;
        // cv::Mat Twc = pKF->GetPoseInverse();
        // Eigen::Isometry3d T1 = ORB_SLAM2::Converter::toSE3Quat(Twc);

        // for (int v = 0; v < DepthOri.rows; v++)
        // {
        //     for (int u = 0; u < DepthOri.cols; u++)
        //     {
        //         // 获取每个像素的深度值
        //         z = DepthOri.at<float>(v, u);
        //         if (z < 0.01 || z > 5)
        //             continue;
        //         x = (u - pKF->cx) * z * pKF->invfx;
        //         y = (v - pKF->cy) * z * pKF->invfy;

        //         p.b = pKF->mimRGBForPC.ptr<uchar>(v)[u * 3];
        //         p.g = pKF->mimRGBForPC.ptr<uchar>(v)[u * 3 + 1];
        //         p.r = pKF->mimRGBForPC.ptr<uchar>(v)[u * 3 + 2];
        //         p.x = x;
        //         p.y = y;
        //         p.z = z;
        //         cloud->push_back(p);
        //     }
        // }
        // pKF->SetPointCloud(cloud);

        // pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
        // mkk++;
        // if (!pKF->mimRGBForPC.empty())
        // {
        //     // pcl::transformPointCloud( *cloud, *cloud2, T1.inverse().matrix());
        //     pcl::transformPointCloud(*cloud, *cloud2, T1.matrix());
        //     cout << "关键帧  : " << mkk << endl;
        //     *mpGlobalCloud += *cloud2;
        //     // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
        //     //  sor.setInputCloud(tmp);
        //     //  sor.setLeafSize(0.1f, 0.1f, 0.1f);
        //     //  //unique_lock<std::mutex> lock(mMutexPC);
        //     //  sor.filter(*mpGlobalCloud);
        //     // pcl::io::savePCDFileBinary("test_pcd25.pcd", *cloud2);
        // }
    }

    void PointCloudMapping::SavePointCloud()
    {

        pcl::io::savePCDFileBinary("test_pcd2.pcd", *mpGlobalCloud);
        cout << "点云数量 ： " << mpGlobalCloud->size() << endl;
    }

}