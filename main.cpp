#include <cmath>

#include "chrono"
#include "iostream"

#include "kdtree.hpp"


int main() {

  KDTree<3, double> tree;
  srand((unsigned int) time(0));

  constexpr size_t iterations = 100000;
  std::vector<Eigen::Vector<double, 3>> points;
  points.reserve(iterations);
  for (size_t iter = 0; iter < iterations; iter++) {
    points.emplace_back(Eigen::Vector<double, 3>::Random());
  }

//  auto start = std::chrono::high_resolution_clock::now();
  for (size_t iter = 0; iter < iterations; iter++) {
    tree.insert(points[iter]);
  }

  auto start = std::chrono::high_resolution_clock::now();
  Eigen::Vector<double, 3> point;
  for (size_t iter = 0; iter < iterations; iter++) {
    point = Eigen::Vector<double, 3>::Random();
    Eigen::Vector<double, 3> v = tree.get_nearest_point(point);
//    Eigen::Vector<double, 3> v = tree.get_leaf_node(point);

//    // validate
//    double min_dist = 1E23;
//    unsigned int min_ind = -1;
//    for (size_t ind = 0; ind < tree.data_.size(); ++ind) {
//      double dist = (point.array() - tree.data_[ind].array()).pow(2).sum();
//      if (dist < min_dist) {
//        min_dist = dist;
//        min_ind = ind;
//      }
//    }
//    double kd_dist = (point.array() - v.array()).pow(2).sum();
//    printf("difference: %f\n", std::fabs(kd_dist - min_dist));

  }


  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations) << " nanoseconds"
            << std::endl;

  return 0;
}
