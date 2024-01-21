#pragma once

#include <vector>
#include <array>
#include <Eigen/Core>


struct KDNode {
  float dim_val{};
  unsigned int left = -1;
  unsigned int right = -1;
  unsigned short dim = 0;
};

template<int SIZE, typename T>

class KDTree {
public:

  KDTree();

  inline Eigen::Vector<T, SIZE> get_nearest_point(const Eigen::Vector<T, SIZE> &point);

  inline void build_tree(const std::vector<Eigen::Vector<T, SIZE>> &point);

  inline KDNode *get_leaf_node(const Eigen::Vector<T, SIZE> &point);

  inline void insert(const Eigen::Vector<T, SIZE> &point);

  std::vector<KDNode> nodes_{};
  std::vector<Eigen::Vector<T, SIZE>> data_{};
  std::vector<KDNode *> stack_;
  std::vector<KDNode *> stack_later_;

};