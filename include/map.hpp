#ifndef __MAP_HPP__
#define __MAP_HPP__

#include <fstream>
#include <memory>
// #include <sstream>
#include <vector>

struct Point {
  int16_t x;
  int16_t y;
  Point(int16_t x, int16_t y) : x(x), y(y){};
  bool operator==(const Point &p) { return (x == p.x && y == p.y); }
};

struct Vertex {
  Point p;
  int16_t g;
  int16_t h;
  int16_t f;
  std::shared_ptr<Vertex> prev;
  Vertex(int16_t x, int16_t y, int16_t g, int16_t h) : p(x, y), g(g), h(h) {
    f = g + h;
  };
  Vertex(Point p, int16_t g, int16_t h) : p(p), g(g), h(h) { f = g + h; };
};

/// @brief
/// Given: Map
///        Starting location
///        Goal location
///        Cost
///  Goal: Find minimum cost path
class Map {
private:
  std::ifstream input_file_;
  std::vector<std::vector<int16_t>> map_;

public:
  /// @brief 
  Map();
  /// @brief 
  /// @param filename 
  Map(std::string filename);

  /// @brief 
  /// @param filename 
  void getMapFromFile(std::string filename);

  /// @brief 
  void printMap();

  /// @brief 
  /// @return 
  std::vector<std::vector<int16_t>> getMap();
};

#endif  // __MAP_HPP__