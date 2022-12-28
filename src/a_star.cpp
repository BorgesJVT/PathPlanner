/*
  Given: Map
         Starting location
         Goal location
         Cost
  Goal: Find minimum cost path
*/

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
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

class Map {
private:
  std::ifstream input_file_;
  std::vector<std::vector<int16_t>> map_;

public:
  Map() {}
  Map(std::string filename) { getMapFromFile(filename); }

  void getMapFromFile(std::string filename) {
    // Try to open the passed file
    input_file_.open(filename, std::ifstream::in);

    // Reading file content
    if (input_file_.is_open()) {
      size_t m, n;
      input_file_ >> m >> n;
      map_.resize(m, std::vector<int16_t>(
                         n, 0)); // Since it is a vector, not an array, it is
                                 // necessary to allocate memory
      std::cout << "Reading... lines: " << m << " and columns: " << n
                << std::endl;
      for (size_t j = 0; j < m; j++)
        for (size_t i = 0; i < n; i++)
          input_file_ >> map_[j][i];
    }
    input_file_.close();
    std::cout << "The file stream has been created. Map uploaded!" << std::endl;
  }

  void printMap() {
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

  std::vector<std::vector<int16_t>> getMap() { return map_; }
};

class Navigation {
private:
  std::vector<std::vector<int16_t>> map_;
  Point current_position_{0, 0};

  void cellSort(std::vector<std::shared_ptr<Vertex>> &open_vertices) {
    std::sort(open_vertices.begin(), open_vertices.end(),
              [](std::shared_ptr<Vertex> a, std::shared_ptr<Vertex> b) {
                return a->f > b->f;
              });
  }

  // Make sure that the cell is in the grid and not occupied
  bool checkValidCell(Point p) {
    // Verify if it is on grid
    if (p.x > -1 && p.y > -1 && p.y < map_.size() && p.x < map_[0].size()) {
      // Now, verify if it a free cell
      if (map_[p.y][p.x] == 0)
        return true;
    }
    return false;
  }

  // Calculate the manhattan distance
  int16_t heuristic(Point init, Point goal) {
    return std::abs(goal.x - init.x) + std::abs(goal.y - init.y);
  }

public:
  Navigation(std::vector<std::vector<int16_t>> map) : map_(map) {}
  Navigation(std::vector<std::vector<int16_t>> map, Point current_pos) : map_(map), current_position_(current_pos) {}

  /*
   * Find a path given the goal point by using an already known map and initial point.
   */
  std::vector<Point> search(Point goal) {
    std::vector<Point> path;

    return search(map_, current_position_, goal);
  }

  /*
   * Find a path given a map passed by reference and the initial and goal points.
   */
  std::vector<Point> search(std::vector<std::vector<int16_t>> &map, Point init,
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

  void printPath(std::vector<Point> path) {
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
};

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