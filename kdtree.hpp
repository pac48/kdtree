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

  inline Eigen::Vector<T, SIZE> get_leaf_node(const Eigen::Vector<T, SIZE> &point);

  inline void insert(const Eigen::Vector<T, SIZE> &point);

private:
  inline KDNode *get_leaf_node_internal(const Eigen::Vector<T, SIZE> &point);

  struct KDNodeDist {
    KDNode *node;
    float dist;
    bool went_left = false;
    bool went_right = false;
  };

  std::vector<KDNode> nodes_{};
  std::vector<KDNodeDist> stack_;
public:
  // DEBUG
  std::vector<Eigen::Vector<T, SIZE>> data_{};
};

