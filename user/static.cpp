#include <vector>
#include <utility>
#include <iostream>

class AmclNode {
public:
  // 定义静态成员变量
  static std::vector<std::pair<int, int>> free_space_indices;

  // 添加自由空间索引的方法
  void addFreeSpaceIndex(int x, int y) {
    free_space_indices.push_back(std::make_pair(x, y));
  }

  // 打印自由空间索引的方法
  void printFreeSpaceIndices() {
    for (const auto & index : free_space_indices) {
      std::cout << "(" << index.first << ", " << index.second << ")" << std::endl;
    }
  }
};

// 在类外部初始化静态成员变量
std::vector<std::pair<int, int>> AmclNode::free_space_indices;

int main() {
  AmclNode node1;
  AmclNode node2;

  // 添加自由空间索引
  node1.addFreeSpaceIndex(1, 2);
  node2.addFreeSpaceIndex(3, 4);

  // 打印自由空间索引
  node1.printFreeSpaceIndices();

  return 0;
}

