#ifndef _KD_TREE_H_
#define _KD_TREE_H_
#include <vector>

template<typename PointT>
class KDTree{
private:
    class Node{
        int    depth;
        PointT value;
        Node*  children[2];
    };
public:
    void insert(const PointT& point){

    }

    std::vector<PointT> findNearPoints(const PointT& target, float maxDistance){

    };
private:
    Node* m_root;
};


#endif