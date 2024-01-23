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
void KDTree<SIZE, T>::build_tree_internal(const std::vector<Eigen::Vector<T, SIZE>> &points, unsigned int dim,
                                          unsigned int start, unsigned int end) {
  std::sort(data_.begin() + start, data_.begin() + end+1,
            [dim](Eigen::Vector<T, SIZE> a, Eigen::Vector<T, SIZE> b) {
              return a[dim] < b[dim];
            });

  auto mid = start + (end - start) / 2;

  if (start == mid && end != mid) {
    nodes_[mid].dim = dim;
    nodes_[mid].dim_val = data_[mid][dim];
    if (data_[end][dim] < data_[mid][dim]) {
      nodes_[mid].left = end;
    } else {
      nodes_[mid].right = end;
    }
    dim = (dim + 1) % SIZE;
    nodes_[end].dim = dim;
    nodes_[end].dim_val = data_[end][dim];
    return;
  } else if (end == start) {
    nodes_[start].dim_val = data_[start][dim];
    nodes_[start].dim = dim;
    return;
  }

  KDNode node;
  node.dim = dim;
  node.dim_val = data_[mid][node.dim];

  node.left = start + ((mid - 1) - start) / 2;
  node.right = (mid + 1) + (end - (mid + 1)) / 2;
//  assert(nodes_[mid].dim_val == 0);
//  assert(node.right != node.left);
  nodes_[mid] = node;
  dim = (dim + 1) % SIZE;
  build_tree_internal(points, dim, start, mid - 1);
  build_tree_internal(points, dim, mid + 1, end);

}

template<int SIZE, typename T>
void KDTree<SIZE, T>::build_tree(const std::vector<Eigen::Vector<T, SIZE>> &points) {
  data_ = points;
  nodes_.resize(points.size());
  build_tree_internal(data_, 0, 0, data_.size() - 1);

//  for (size_t i = 0; i < data_.size(); i++) {
//    KDNode &node = nodes_[i];
//    assert(data_[i][node.dim] == node.dim_val);
//  }
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
  int o = 0;
}

template<int SIZE, typename T>
Eigen::Vector<T, SIZE> KDTree<SIZE, T>::get_nearest_point(const Eigen::Vector<T, SIZE> &point) {
  assert(!nodes_.empty());
  unsigned int min_index = data_.size() / 2 - 1;
  float min_dist = (point.array() - data_[min_index].array()).pow(2).sum();
  get_nearest_point_recurse(point, min_dist, min_index,  min_index);
  return data_[min_index];

}


template
class KDTree<3, float>;

template
class KDTree<3, double>;
