#pragma once

#include "kdtree.hpp"

template<int SIZE, typename T>
KDTree<SIZE, T>::KDTree() {
  nodes_.reserve(1024);
  data_.reserve(1024);
}

//template<int SIZE>
//void KDTree<SIZE>::insert(const Eigen::Vector<double, SIZE> &point) {
//  unsigned int dim = -1;
//  unsigned int index = 0;
//  unsigned int last_index = 0;
//  bool cond;
//  while (index < nodes_.size()) {
//    dim++;
//    KDNode &node = nodes_[index];
//    last_index = index;
//    cond = point[dim % SIZE] < node.dim_val;
//    if (cond) {
//      index = node.left;
//    } else {
//      index = node.right;
//    }
//  }
//
//  index = nodes_.size();
//  if (!nodes_.empty()) {
//    KDNode &node = nodes_[last_index];
//    if (cond) {
//      node.left = index;
//    } else {
//      node.right = index;
//    }
//  }
//
//  nodes_.push_back({point[dim % SIZE], (unsigned int) -1, (unsigned int) -1});
//  data_.push_back(point);
//}



template<int SIZE, typename T>
void KDTree<SIZE, T>::insert(const Eigen::Vector<T, SIZE> &point) {
  KDNode *node;
  unsigned int index = 0;
  while (index < nodes_.size()) {
    node = &nodes_[index];
    if (point[node->dim] < node->dim_val) {
      index = node->left;
    } else {
      index = node->right;
    }
  }

  unsigned int dim;
  if (!nodes_.empty()) {
    if (point[node->dim] < node->dim_val) {
      node->left = nodes_.size();
    } else {
      node->right = nodes_.size();
    }
    dim = (node->dim + 1) % SIZE;
  } else {
    dim = 0;
  }

  nodes_.push_back({(float) point[dim], (unsigned int) -1, (unsigned int) -1, (unsigned short) (dim)});
  data_.push_back(point);
}



//template
//class KDTree<2>;

template
class KDTree<3, float>;

template
class KDTree<3, double>;

//template
//class KDTree<4>;
