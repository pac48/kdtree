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
  inline unsigned int get_leaf_node_internal(const Eigen::Vector<T, SIZE> &point);

  void build_tree_internal(const std::vector<Eigen::Vector<T, SIZE>> &points, unsigned int dim, unsigned int start,
                           unsigned int end);

  inline void get_nearest_point_recurse(const Eigen::Vector<T, SIZE> &point, float &min_dist, unsigned int &min_index,
                                        unsigned int index);

  std::vector<KDNode> nodes_{};
  size_t root_ = -1;
public:
  // DEBUG
  std::vector<Eigen::Vector<T, SIZE>> data_{};
};

