#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H


#include <pcl/point_types.h>
#include <iostream>
#include<vector>
#include "MapPoint.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include<vtkAutoInit.h>

VTK_MODULE_INIT(vtkRenderingOpenGL2);

VTK_MODULE_INIT(vtkInteractionStyle);

VTK_MODULE_INIT(vtkRenderingFreeType);

//避免和Eigen冲突
#ifdef Success 
#undef Success
#endif



class KeyFrame;

namespace ORB_SLAM2
{
class PointCloudMapping
{
    
public:
    //构造函数
    PointCloudMapping();
    //显示点云
    void DisplayPointCloud();

    //插入关键帧
    void InsertKeyFrame(KeyFrame *pKF);

    //更新点云
    void UpdatePointCloud();

    //保存点云
    void SavePointCloud();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr GenerateCloud(KeyFrame *pKF);

public:
    cv::Mat mimDepthForPC; 
    std::vector<cv::Mat> mvimDepthForPC;
    std::vector<cv::Mat> mvimDepthOriForPC;
    std::vector<KeyFrame*> mvpKF;
    // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mpGlobalCloud;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mpGlobalCloud;

    bool mflag ;

    std::mutex mMutexPC;
    std::mutex mMutexPCKF;
    int mkk ;


};






}


#endif