#ifndef _CLUSTER_H_
#define _CLUSTER_H_
#include "render/box.h"

#include <vector>
#include <limits>
#include <unordered_set>


template<typename PointT>
std::vector<int> nearPoints(typename pcl::PointCloud<PointT>::Ptr cloud, int pointIndex, float proximity){
    std::vector<int> result;

    PointT point = cloud->at(pointIndex);
    
    for(int i = 0; i < cloud->size(); i++){
        PointT p = cloud->at(i);
        if(  i != pointIndex && ( ( (point.x-p.x)*(point.x-p.x) + (point.y-p.y)*(point.y-p.y) + (point.z-p.z)*(point.z-p.z)  ) < proximity)){
            result.push_back(i);
        }
    }

    return result;
}


template<typename PointT>
struct ClusterContext{
    std::vector<int>         clusterMembers;
    std::unordered_set<int>& processed;
    float                    proximity;
    typename pcl::PointCloud<PointT>::Ptr cloud;
    
    ClusterContext(std::unordered_set<int>& p):
        processed(p){
    }
};

template<typename PointT>
void clusterPoint(ClusterContext<PointT>& context,int pointIndex){
    context.processed.insert(pointIndex);
    std::vector<int> near = nearPoints<PointT>(context.cloud,pointIndex,context.proximity);
    
    for(int p = 0; p < near.size(); p++){
        if(context.processed.find(near[p]) == context.processed.end()){
            context.clusterMembers.push_back(near[p]);
            clusterPoint(context,near[p]);
        }
    }
}




template<typename PointT>
std::vector<Box> cluster(typename pcl::PointCloud<PointT>::Ptr cloud,float proximity){
    std::vector<Box>              clusters;
    std::unordered_set<int>       processed;
    
    for(int i = 0; i < cloud->size(); i++){
    //    std::cout << "clustering " << cloud->at(i) << std::endl;

        ClusterContext<PointT> context(processed);
        context.cloud      = cloud;
        context.proximity  = proximity;

        if( processed.find(i) == processed.end() ){
            clusterPoint(context,i);
            Box clusterBounds;
            clusterBounds.x_min = std::numeric_limits<float>::max();
            clusterBounds.y_min = std::numeric_limits<float>::max();
            clusterBounds.z_min = std::numeric_limits<float>::max();
            clusterBounds.x_max = std::numeric_limits<float>::max() * -1;
            clusterBounds.y_max = std::numeric_limits<float>::max() * -1;
            clusterBounds.z_max = std::numeric_limits<float>::max() * -1;
             
            for(int p = 0; p < context.clusterMembers.size(); p++){
                PointT point = cloud->at(context.clusterMembers[p]);
                if(point.x > clusterBounds.x_max){ clusterBounds.x_max = point.x; }
                if(point.y > clusterBounds.y_max){ clusterBounds.y_max = point.y; }
                if(point.z > clusterBounds.z_max){ clusterBounds.z_max = point.z; }

                if(point.x < clusterBounds.x_min){ clusterBounds.x_min = point.x; }
                if(point.y < clusterBounds.y_min){ clusterBounds.y_min = point.y; }
                if(point.z < clusterBounds.z_min){ clusterBounds.z_min = point.z; }              
            }
            if(context.clusterMembers.size() > 15){
                clusters.push_back(clusterBounds);
            }
        }
    }
    return clusters;
}




#endif
