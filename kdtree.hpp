#pragma once

#include <vector>
#include <array>
#include <Eigen/Core>


struct KDNode {
  float dim_val;
  unsigned int left = -1;
  unsigned int right = -1;
  unsigned short dim = 0;
};

template<int SIZE, typename T>

class KDTree {
public:

  KDTree();

  void insert(const Eigen::Vector<T, SIZE> &point);


public:
  std::vector<KDNode> nodes_{};
  std::vector<Eigen::Vector<T, SIZE>> data_{};
};