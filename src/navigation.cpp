#include <algorithm>
#include <iostream>
#include "navigation.hpp"

void Navigation::cellSort(std::vector<std::shared_ptr<Vertex>> &open_vertices) {
  std::sort(open_vertices.begin(), open_vertices.end(),
            [](std::shared_ptr<Vertex> a, std::shared_ptr<Vertex> b) {
              return a->f > b->f;
            });
}

bool Navigation::checkValidCell(Point p) {
  // Verify if it is on grid
  if (p.x > -1 && p.y > -1 && p.y < map_.size() && p.x < map_[0].size()) {
    // Now, verify if it a free cell
    if (map_[p.y][p.x] == 0)
      return true;
  }
  return false;
}

int16_t Navigation::heuristic(Point init, Point goal) {
  return std::abs(goal.x - init.x) + std::abs(goal.y - init.y);
}

Navigation::Navigation(std::vector<std::vector<int16_t>> map) : map_(map) {}
Navigation::Navigation(std::vector<std::vector<int16_t>> map, Point current_pos) : map_(map), current_position_(current_pos) {}

std::vector<Point> Navigation::search(Point goal) {
  std::vector<Point> path;

  return search(map_, current_position_, goal);
}

std::vector<Point> Navigation::search(std::vector<std::vector<int16_t>> &map, Point init,
                          Point goal) {
  if (map[init.y][init.x] != 0 || map[goal.y][goal.x] != 0) {
    std::cout << "There are no path available!" << std::endl;
    return {};
  }
  
  std::vector<Point> path;
  // Create the vector of open cells
  std::vector<std::shared_ptr<Vertex>> open_vertices{};
  // Create the vector of closed cells
  std::vector<std::shared_ptr<Vertex>> closed_vertices{};

  // Initialize the starting cell
  int16_t h = heuristic(init, goal);

  // Add initial vertex to open list
  std::shared_ptr<Vertex> initial = std::make_shared<Vertex>(init, 0, h);
  initial->prev = nullptr;
  open_vertices.push_back(initial);

  std::shared_ptr<Vertex> current_v;
  while (!open_vertices.empty()) {
    // sort the open list by comparing the 'f=g+h'
    cellSort(open_vertices);
    // take the smallest 'f' element 
    current_v = open_vertices.back();
    // DEBUG
    // std::cout << "current: " << current_v->p.x << " "  << current_v->p.y << std::endl;
    // If neighbor is the goal, stop search
        if (current_v->p == goal)
          break;

    // pop current_v off the open list
    open_vertices.pop_back();
    // Expand vector of open vertices with valid neighbors of current v
    std::vector<Point> neighbors{Point(current_v->p.x + 1, current_v->p.y),
                                  Point(current_v->p.x, current_v->p.y + 1),
                                  Point(current_v->p.x - 1, current_v->p.y),
                                  Point(current_v->p.x, current_v->p.y - 1)};
    
    for (auto neighbor : neighbors) {
      // Check if neighbors are valid cells
      if (checkValidCell(neighbor)) {
        std::shared_ptr<Vertex> neighbor_vertex = std::make_shared<Vertex>(
            neighbor, current_v->g + 1, heuristic(neighbor, goal));
        neighbor_vertex->prev = current_v;
        open_vertices.push_back(neighbor_vertex);
      }
    }
    closed_vertices.push_back(current_v);
  } // end while

  while (current_v != nullptr) {
    path.push_back(current_v->p);
    current_v = current_v->prev;
  }
  
  return path;
}

void Navigation::printPath(std::vector<Point> path) {
  if (map_.empty()) {
    std::cout << "The map is empty!" << std::endl;
    return;
  }
  std::vector<std::vector<int16_t>> map = map_;

  for (Point p : path)
    map[p.y][p.x] = -1;

  std::cout << "---------------------" << std::endl; 
  for (size_t j = 0; j < map.size(); j++) {
    for (size_t i = 0; i < map[j].size(); i++) {
      if (map[j][i] == 0)
        std::cout << '0' << ' ';
      else if(map[j][i] == 1)
        std::cout << 'X' << ' ';
      else if(map[j][i] == -1)
        std::cout << '*' << ' ';
    }
    std::cout << std::endl;
  }
  std::cout << "---------------------" << std::endl; 
}