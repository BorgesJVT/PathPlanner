#include <iostream>
#include "map.hpp"

Map::Map() {}
Map::Map(std::string filename) { getMapFromFile(filename); }

void Map::getMapFromFile(std::string filename) {
  // Try to open the passed file
  input_file_.open(filename, std::ifstream::in);

  // Reading file content
  if (input_file_.is_open()) {
    size_t m, n;
    input_file_ >> m >> n;
    map_.resize(
        m, std::vector<int16_t>(n, 0)); // Since it is a vector, not an array,
                                        // it is necessary to allocate memory
    std::cout << "Reading... lines: " << m << " and columns: " << n
              << std::endl;
    for (size_t j = 0; j < m; j++)
      for (size_t i = 0; i < n; i++)
        input_file_ >> map_[j][i];
  }
  input_file_.close();
  std::cout << "The file stream has been created. Map uploaded!" << std::endl;
}

void Map::printMap() {
  if (map_.empty()) {
    std::cout << "The map is empty!" << std::endl;
    return;
  }

  std::cout << "---------------------" << std::endl;
  for (size_t j = 0; j < map_.size(); j++) {
    for (size_t i = 0; i < map_[j].size(); i++) {
      if (map_[j][i])
        std::cout << 'X' << ' '; // X for blocked
      else
        std::cout << '0' << ' '; // 0 for free pass
    }
    std::cout << std::endl;
  }
  std::cout << "---------------------" << std::endl;
}

std::vector<std::vector<int16_t>> Map::getMap() { return map_; }