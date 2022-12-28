#include "navigation.hpp"
#include "map.hpp"

int main() {
  Map map;
  map.getMapFromFile("../data/map.data");
  map.printMap();

  Navigation navigation(map.getMap());
  std::vector<std::vector<int16_t>> grid = map.getMap();
  // cannot pass map.getMap() as argument because the arg receives a reference
  // and map.getMap() cannot be changed
  // std::vector<Point> path = navigation.search(grid, Point(0, 0), Point(5, 4));

  std::vector<Point> path = navigation.search(Point(5, 2));
  navigation.printPath(path);

  return 0;
}