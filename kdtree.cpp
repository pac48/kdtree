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
Eigen::Vector<T, SIZE> KDTree<SIZE, T>::get_nearest_point(const Eigen::Vector<T, SIZE> &point) {
  assert(!nodes_.empty());
  float min_dist = (point.array() - data_[0].array()).pow(2).sum();
  unsigned int ind = 0;
  float dist = 0;
  unsigned int ind_dist = -1;

  stack_.reserve(128);
  stack_.push_back({nodes_[0], 0});
  while (!stack_.empty()) {
    if (stack_.back().dist > min_dist || (stack_.back().went_right && stack_.back().went_left)) {
      stack_.pop_back();
      continue;
    }
    KDNode &node = stack_.back().node;

    const auto add_left = [this, node, point](
        std::vector<KDNodeDist> &stack, float near_dist) {
      if (node.left < nodes_.size()) {
        stack.push_back({nodes_[node.left], near_dist});
        return std::make_tuple<float, const unsigned int>(
            (float) (point.array() - data_[node.left].array()).pow(2).sum(), (unsigned int) node.left);
      }
      return std::make_tuple<float, unsigned int>(0, -1);
    };
    const auto add_right = [this, node, point](
        std::vector<KDNodeDist> &stack, float near_dist) {
      if (node.right < nodes_.size()) {
        stack.push_back({nodes_[node.right], near_dist});
        return std::make_tuple<float, unsigned int>((float) (point.array() - data_[node.right].array()).pow(2).sum(),
                                                    (unsigned int) node.right);
      }
      return std::make_tuple<float, const unsigned int>(0, -1);
    };

    if (point[node.dim] < node.dim_val && !stack_.back().went_left) {
      stack_.back().went_left = true;
      std::tie(dist, ind_dist) = add_left(stack_, 0.0);
    } else if (point[node.dim] >= node.dim_val && !stack_.back().went_right) {
      stack_.back().went_right = true;
      std::tie(dist, ind_dist) = add_right(stack_, 0.0);
    } else {
      const float near_dist = (node.dim_val - point[node.dim]) * (node.dim_val - point[node.dim]);
      if (!stack_.back().went_left && near_dist < min_dist) {
        stack_.back().went_left = true;
        std::tie(dist, ind_dist) = add_left(stack_, near_dist);
      } else if (!stack_.back().went_right && near_dist < min_dist) {
        stack_.back().went_right = true;
        std::tie(dist, ind_dist) = add_right(stack_, near_dist);
      } else {
        stack_.back().went_left = true;
        stack_.back().went_right = true;
      }
    }
    if (ind_dist < nodes_.size() && dist < min_dist) {
      min_dist = dist;
      ind = ind_dist;
    }
  }

  return data_[ind];

}


template
class KDTree<3, float>;

template
class KDTree<3, double>;
