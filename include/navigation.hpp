#ifndef __NAVIGATION_HPP__
#define __NAVIGATION_HPP__

#include "map.hpp"

class Navigation {
private:
  std::vector<std::vector<int16_t>> map_;
  Point current_position_{0, 0};

  /// @brief Sort a vector of vertices based on its heuristic cost 'f'.
  /// @param open_vertices 
  void cellSort(std::vector<std::shared_ptr<Vertex>> &open_vertices);
  
  /// @brief Check if the cell is on the grid and not occupied.
  /// @param p 
  /// @return 
  bool checkValidCell(Point p);
  
  /// @brief Calculate the manhattan distance given an initial and goal points.
  /// @param init 
  /// @param goal 
  /// @return 
  int16_t heuristic(Point init, Point goal);

public:
  /// @brief Navigation constructor which receives a 2D vector map.
  /// @param map
  Navigation(std::vector<std::vector<int16_t>> map);
  /// @brief Navigation constructor which receives a 2D vector map
  /// and the a reference point to start the navigation (initial point).
  /// @param map
  /// @param current_pos
  Navigation(std::vector<std::vector<int16_t>> map, Point current_pos);

  /// @brief Find a path given the goal point by using
  /// an already known map and initial point.
  /// @param goal
  /// @return
  std::vector<Point> search(Point goal);

  /// @brief Find a path given a map passed by reference and 
  /// the initial and goal points.
  /// @param map 
  /// @param init 
  /// @param goal 
  /// @return 
  std::vector<Point> search(std::vector<std::vector<int16_t>> &map, Point init,
                            Point goal);

  /// @brief Print the path over the already known map on the screen. 
  /// @param path 
  void printPath(std::vector<Point> path);
};

#endif  // __NAVIGATION_HPP__