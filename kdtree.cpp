#pragma once

#include "kdtree.hpp"

template<int SIZE, typename T>
KDTree<SIZE, T>::KDTree() {
  nodes_.reserve(1024);
  data_.reserve(1024);
  stack_.reserve(1024);
  stack_later_.reserve(1024);

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
Eigen::Vector<T, SIZE> KDTree<SIZE, T>::get_nearest_point(const Eigen::Vector<T, SIZE> &point) {
  assert(!nodes_.empty());

  float min_dist = (point.array() - data_[0].array()).pow(2).sum();
  unsigned int ind = 0;
//  std::vector<KDNode*> stack = {&nodes_[nodes_[0].left], &nodes_[nodes_[0].right]};


  stack_.push_back(&nodes_[0]);
  int total_push = 0;
  while (!stack_.empty() || !stack_later_.empty()) {
    if (stack_.empty()) {
      stack_ = stack_later_;
      stack_later_.clear();
    }
    KDNode *node = stack_.back(); // this dist has already been checked
    stack_.pop_back();
    if (std::fabs((node->dim_val - point[node->dim]) * (node->dim_val - point[node->dim])) > min_dist) {
      continue;
    }

    const auto add_left = [this, node, &point, &min_dist, &ind, &total_push](std::vector<KDNode *> &stack) {
      if (node->left < nodes_.size()) {
        KDNode *left_node = &nodes_[node->left];
        stack.push_back(left_node);
        total_push++;
        float dist = (point.array() - data_[node->left].array()).pow(2).sum();
        if (dist <= min_dist) {
          min_dist = dist;
          ind = node->left;
        }
      }
    };
    const auto add_right = [this, node, &point, &min_dist, &ind, &total_push](std::vector<KDNode *> &stack) {
      if (node->right < nodes_.size()) {
        KDNode *right_node = &nodes_[node->right];
        stack.push_back(right_node);
        total_push++;
        float dist = (point.array() - data_[node->right].array()).pow(2).sum();
        if (dist <= min_dist) {
          min_dist = dist;
          ind = node->right;
        }
      }
    };

    bool add_both = std::fabs((node->dim_val - point[node->dim]) * (node->dim_val - point[node->dim])) < min_dist;
    if (point[node->dim] < node->dim_val) {
      add_left(stack_);
      if (add_both) {
        add_right(stack_later_);
      }
    } else {
      add_right(stack_);
      if (add_both) {
        add_left(stack_later_);
      }
    }

  }

//  printf("total_push: %d\n", total_push);

  return data_[ind];

}


template
class KDTree<3, float>;

template
class KDTree<3, double>;
