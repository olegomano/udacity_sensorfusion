#ifndef _KD_TREE_H_
#define _KD_TREE_H_
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <vector>
#include <math.h>  


template<typename PointT>
class KDTree{
public:
    struct Node{
        const  int depth;
        int    id;
        float  split;
        Node*  children[2];

        Node(int d) : depth(d){
            children[0] = nullptr;
            children[1] = nullptr;
            id          = -1;
        }
    };
public:
    KDTree(typename pcl::PointCloud<PointT>::Ptr cloud){
        auto startTime = std::chrono::steady_clock::now();


        std::vector<PointT> start;
        for(int i = 0; i < cloud->size(); i++){
            start.push_back(cloud->at(i));
        }
        m_root = new Node(0);
        split(m_root,start);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "Building KDTree took " << elapsedTime.count() << " milliseconds" << std::endl;
    }

    typename pcl::PointCloud<PointT>::Ptr nearPoints(const PointT& point, float width){
        typename pcl::PointCloud<PointT>::Ptr result(new pcl::PointCloud<PointT>());
        return result;
    }

    std::vector<int> nearIndexes(const PointT& point, float width){
        
    }

private:
    void split(Node* self,const std::vector<PointT>& points){//make points ints, and make them indexes
        if(points.size() == 1){
            self->id = 0; //todo: make indexes correct
            return;
        }
        
        int depth   = self->depth;
        int samples = 100;
        if(points.size() < 100){
            samples = points.size();
        }

        self->split = 0;
        for(int i  = 0; i < points.size(); i++){
            int pointIndex = rand() % points.size();
            self->split+=pointValue(points[pointIndex],self->depth);
        }
        self->split/=samples;

        std::vector<PointT> left;
        std::vector<PointT> right;
        for(int i = 0; i < points.size(); i++){
            float value = pointValue(points[i],self->depth);
            if(value < self->split){
                left.push_back(points[i]);
            }else{
                right.push_back(points[i]);
            }
        }

        Node* leftNode  = new Node(self->depth + 1);
        Node* rightNode = new Node(self->depth + 1); 
        self->children[0] = leftNode;
        self->children[1] = rightNode;
        
        split(leftNode,left);
        split(rightNode,right);
    }

    float pointValue(const PointT& point, int depth){
        switch(depth%3){
            case 0:
                return point.x;
            case 1:
                return point.y;
            case 2:
                return point.z;
        }
        return -1;
    }
 private:
    Node* m_root;
    typename pcl::PointCloud<PointT>::Ptr m_cloud;
};


#endif