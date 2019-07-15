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
#include <assert.h>
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
    KDTree(typename pcl::PointCloud<PointT>::Ptr cloud){ //balanced kd-tree
        auto startTime = std::chrono::steady_clock::now();

        m_cloud = cloud;
        std::vector<int> indexList;
        for(int i = 0; i < cloud->size(); i++){
            indexList.push_back(i);
        }
        m_root = new Node(0);
        split(m_root,indexList);

        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "Building KDTree took " << elapsedTime.count() << " milliseconds" << std::endl;
        std::cout << "Empty Split count : " << m_emptySplits << std::endl;
    }

    std::vector<int> nearIndexes(const PointT& point, float width){
        std::vector<int> result;
        findNearChildren(m_root,point,width,result);
        return result;
    }

private:
    void split(Node* self,const std::vector<int>& indexes){//make points ints, and make them indexes
        if(indexes.size() == 1){
            self->id = indexes[0]; 
            return;
        }
        self->id = -1;
        if(indexes.size() == 0){
            ++m_emptySplits;
            return;
        }
        
        int depth   = self->depth;
        int samples = 200;
        self->split = 0;

        if(indexes.size() < samples){
            for(int i = 0; i < indexes.size(); i++){
                float value = pointValue(m_cloud->at(indexes[i]),depth);
                self->split+= value;     
            }
            self->split /= indexes.size();
        }else{
            for(int i  = 0; i < samples; i++){
                int pointIndex = rand() % indexes.size();
                self->split+=pointValue(m_cloud->at(indexes[pointIndex]),depth);
            }
            self->split/=samples;
        }
       
        std::vector<int> left;
        std::vector<int> right;
        for(int i = 0; i < indexes.size(); i++){
            float value = pointValue(m_cloud->at(indexes[i]),depth);
            if(value < self->split){
                left.push_back(indexes[i]);
            }else{
                right.push_back(indexes[i]);
            }
        }

        Node* leftNode  = new Node(self->depth + 1);
        Node* rightNode = new Node(self->depth + 1); 
        self->children[0] = leftNode;
        self->children[1] = rightNode;
        split(leftNode,left);
        split(rightNode,right);
    }

    void findNearChildren(Node* node,  const PointT& point, float width,std::vector<int>& nearIndexes){
        Node* candidates[2];
        candidates[0]=nullptr;
        candidates[1]=nullptr;

        if(node->id != -1){
            PointT candidate = m_cloud->at(node->id);
            float  distance  = (point.x - candidate.x)*(point.x - candidate.x) + (point.y - candidate.y)*(point.y - candidate.y) + (point.z - candidate.z)*(point.z - candidate.z); 
            if(distance < width*width){
                nearIndexes.push_back(node->id);
            }
            return;
        }

        candidateChildren(node,point,width,candidates);
        if(candidates[0] != nullptr){findNearChildren(candidates[0], point, width, nearIndexes);}
        if(candidates[1] != nullptr){findNearChildren(candidates[1], point, width, nearIndexes);}

    }

    void candidateChildren(Node* node, const PointT& point, const float width,Node** candidates){
        float value = pointValue(point,node->depth);
        float min   = value - width;
        float max   = value + width;
        if(min < node->split){
            candidates[0] = node->children[0];
        }
        if(max > node->split){
            candidates[1] = node->children[1];
        } 
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
        static_assert(true,"Should not happen");
        return -1;
    }
 private:
    int   m_emptySplits = 0;
    Node* m_root;
    typename pcl::PointCloud<PointT>::Ptr m_cloud;
};


#endif