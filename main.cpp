#include "chrono"
#include "iostream"

#include "kdtree.hpp"


int main() {

  KDTree<3, double> tree;
  Eigen::Vector<double, 3> point;
  srand((unsigned int) time(0));

  constexpr size_t iterations = 128 * 128 * 128;
  auto start = std::chrono::high_resolution_clock::now();
  for (size_t iter = 0; iter < iterations; iter++) {
    point = Eigen::Vector<double, 3>::Random();
    tree.insert(point);
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);
  std::cout << "Time taken by function: " << (double) duration.count() << " nanoseconds" << std::endl;
  std::cout << "Average: " << ((double) duration.count()) / (iterations) << " nanoseconds"
            << std::endl;

  std::cout << tree.data_.back() << "\n";
  std::cout << tree.data_.size() << "\n";
  std::cout << tree.nodes_.size() << "\n";

  return 0;
}
