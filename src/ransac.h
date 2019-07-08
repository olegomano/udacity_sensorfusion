#ifndef _RANSAC_H_
#define _RANSAC_H_


#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <stdlib.h>
#include <math.h>  


template<typename PointT>
PointT abs(const PointT& a){
    PointT result;
    result.x = abs(a.x);
    result.y = abs(a.x);
    result.z = abs(a.x);
    return result;
}

template<typename PointT>
PointT crossProduct(const PointT& a, const PointT& b){
    PointT result;
    result.x = a.y*b.z - a.z*b.y;
    result.y = a.x*b.z - a.z*b.x;
    result.z = a.x*b.y - a.y*b.x;    
    return result;
}

template<typename PointT>
PointT sub(const PointT& a, const PointT& b){
    PointT result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;    
    return result;
}

template<typename PointT>
float dot(const PointT& a, const PointT& b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

template<typename PointT>
float distancePlanePoint(const PointT& point, const PointT& normal, const PointT& origin){
    PointT vec = sub<PointT>(point,origin);
    float  d   = dot<PointT>(vec,normal);
    return abs(d / sqrt(dot<PointT>(normal,normal)));
}


template<typename PointT>
std::pair<PointT,PointT> estimateModel(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
    int    maxInliers = 0;
    PointT modelOrigin;
    PointT modelNormal;

    std::cout << "PointCloud size " << cloud->size() << std::endl;

    for(int i = 0; i < maxIterations; i++){
        size_t p1 = rand() % cloud->size();
        size_t p2 = rand() % cloud->size();
        size_t p3 = rand() % cloud->size();

        if(p1 == p2 || p2 == p3 || p3 == p1) {
            maxIterations--;
            continue;
        }

        PointT origin = cloud->at(p1);
        PointT normal = crossProduct<PointT>( sub<PointT>(cloud->at(p2),cloud->at(p1)), sub<PointT>(cloud->at(p3),cloud->at(p1)));

        int inlierCount = 0;

        for(size_t p = 0; p < cloud->size(); p++){
            if( distancePlanePoint<PointT>(cloud->at(p),normal,origin) <= distanceThreshold){
                inlierCount++;
            }
        }

        if(inlierCount > maxInliers){
            maxInliers  = inlierCount;
            modelOrigin = origin;
            modelNormal = normal;
        }

    }

    std::pair<PointT,PointT> model(modelNormal,modelOrigin);
    return model;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold){
    auto startTime = std::chrono::steady_clock::now();

    std::pair<PointT,PointT> modelParams = estimateModel<PointT>(cloud,maxIterations,distanceThreshold);

    typename pcl::PointCloud<PointT>::Ptr floor(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstacle(new pcl::PointCloud<PointT>());
    
    for(size_t i = 0; i < cloud->size(); i++){
        if( distancePlanePoint<PointT>(cloud->at(i),modelParams.first,modelParams.second) <= distanceThreshold){
            floor->push_back(cloud->at(i));
        }else{
            obstacle->push_back(cloud->at(i));
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ransac took " << elapsedTime.count() << " ms" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> result(floor,obstacle);
    return result;
}

#endif