#pragma once

#include "kdtree.hpp"

template<int SIZE, typename T>
KDTree<SIZE, T>::KDTree() {}

template<int SIZE, typename T>
Eigen::Vector<T, SIZE> KDTree<SIZE, T>::get_leaf_node(const Eigen::Vector<T, SIZE> &point) {
  return data_[get_leaf_node_internal(point)];
}


template<int SIZE, typename T>
unsigned int KDTree<SIZE, T>::get_leaf_node_internal(const Eigen::Vector<T, SIZE> &point) {
  unsigned int index_new = 0;
  unsigned int index = 0;
  while (index_new < nodes_.size()) {
    index = index_new;
    KDNode &node = nodes_[index_new];
    if (point[node.dim] < node.dim_val) {
      index_new = node.left;
    } else {
      index_new = node.right;
    }
  }
  return index;
}

template<int SIZE, typename T>
void KDTree<SIZE, T>::insert(const Eigen::Vector<T, SIZE> &point) {
  unsigned int dim = 0;
  if (!nodes_.empty()) {
    KDNode &node = nodes_[get_leaf_node_internal(point)];
    if (point[node.dim] < node.dim_val) {
      node.left = nodes_.size();
    } else {
      node.right = nodes_.size();
    }
    dim = (node.dim + 1) % SIZE;
  }
  nodes_.push_back({(float) point[dim], (unsigned int) -1, (unsigned int) -1, (unsigned short) (dim)});
  data_.push_back(point);
}


template<int SIZE, typename T>
void
KDTree<SIZE, T>::get_nearest_point_recurse(const Eigen::Vector<T, SIZE> &point, float &min_dist,
                                           unsigned int &min_index, unsigned int index) {
  KDNode &node = nodes_[index];
  float pot_dist = (point.array() - data_[index].array()).pow(2).sum();
  if (pot_dist < min_dist) {
    min_dist = pot_dist;
    min_index = index;
  }

  if (point[node.dim] < node.dim_val) {
    // go left
    if (node.left < nodes_.size()) {
      get_nearest_point_recurse(point, min_dist, min_index, node.left);
    }
    if ((node.dim_val - point[node.dim]) * (node.dim_val - point[node.dim]) < min_dist && node.right < nodes_.size()) {
      // go right
      get_nearest_point_recurse(point, min_dist, min_index, node.right);
    }
  } else {
    // go right
    if (node.right < nodes_.size()) {
      get_nearest_point_recurse(point, min_dist, min_index, node.right);
    }
    if ((node.dim_val - point[node.dim]) * (node.dim_val - point[node.dim]) < min_dist && node.left < nodes_.size()) {
      // go left
      get_nearest_point_recurse(point, min_dist, min_index, node.left);
    }
  }
}

template<int SIZE, typename T>
Eigen::Vector<T, SIZE> KDTree<SIZE, T>::get_nearest_point(const Eigen::Vector<T, SIZE> &point) {
  assert(!nodes_.empty());
  float min_dist = (point.array() - data_[0].array()).pow(2).sum();
  unsigned int min_index = 0;
  get_nearest_point_recurse(point, min_dist, min_index, 0);
  return data_[min_index];

}


template
class KDTree<3, float>;

template
class KDTree<3, double>;
