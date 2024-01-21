#pragma once

#include "kdtree.hpp"

template<int SIZE, typename T>
KDTree<SIZE, T>::KDTree() {
  nodes_.reserve(1024);
  data_.reserve(1024);
}

template<int SIZE, typename T>
KDNode *KDTree<SIZE, T>::get_leaf_node(const Eigen::Vector<T, SIZE> &point) {
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
  return node;
}

template<int SIZE, typename T>
void KDTree<SIZE, T>::insert(const Eigen::Vector<T, SIZE> &point) {
  KDNode *node = get_leaf_node(point);
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



template<int SIZE, typename T>
void KDTree<SIZE, T>::build_tree(const std::vector<Eigen::Vector<T, SIZE>> &points) {
  data_ = points;
  nodes_.reserve(points.size());
  std::vector<std::array<unsigned int, 3>> stack = {{0, (unsigned int) data_.size() - 1, 0}};

  while (!stack.empty()) {
    auto range = stack.back();
    stack.pop_back();
    if (range[1] - range[0] <= 2) {
      continue;
    }
    unsigned int dim = range[2];
    std::sort(data_.begin() + range[0], data_.begin() + range[1],
              [dim](Eigen::Vector<T, SIZE> a, Eigen::Vector<T, SIZE> b) {
                return a[dim] < b[dim];
              });
    KDNode node;
    auto mid = range[0] / 2 + range[1] / 2;
    node.dim = dim;
    node.left = range[0];
    node.right = mid;
    node.dim_val = data_[node.right][node.dim_val];
    nodes_.emplace_back(node);

    dim = (dim + 1) % SIZE;
    stack.push_back({range[0], mid - 1, dim});
    stack.push_back({mid, range[1], dim});
  }

}



template<int SIZE, typename T>
const Eigen::Vector<T, SIZE> & KDTree<SIZE, T>::get_nearest_point(const Eigen::Vector<T, SIZE> &point) {
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

  return {};

}


template
class KDTree<3, float>;

template
class KDTree<3, double>;
