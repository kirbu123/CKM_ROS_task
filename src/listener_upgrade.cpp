#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <std_msgs/Float64.h>
#include <vector>

// global space
ros::Publisher path_pub;
ros::Publisher time_pub;
std::vector<int> grid_map;
nav_msgs::OccupancyGrid grid_map_obj;
geometry_msgs::PoseStamped goal;
nav_msgs::Odometry odom;
int width = 0;
int height = 0;

// function for updating map grid
void MapUpdater(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  grid_map_obj = *msg;
  grid_map.assign(msg->data.begin(), msg->data.end());
  width = msg->info.width;
  height = msg->info.height;
  /*
  std::cout << "GET GRID: " << "width: " << width << ' ' << "height: " << height << '\n';
  */
}

// function for transform from idx to grid x, y
std::pair<int, int> ToPare(const int& idx) {
  std::pair<int, int> result;
  result.first = idx - (idx / width) * width;
  result.second = idx / width;
  return result;
}

// function for transform from grid x, y to map idx
int ToIndex(const int& x, const int& y) {
  return x + y * width;
}

// function for transform from map idx to real x, y
std::pair<double, double> ToXY(const int& idx) {
  std::pair<double, double> result;
  double x = ToPare(idx).first;
  double y = ToPare(idx).second;
  double resolution = grid_map_obj.info.resolution;
  double origin_x = grid_map_obj.info.origin.position.x;
  double origin_y = grid_map_obj.info.origin.position.y;
  result.first = x * resolution + origin_x;
  result.second = y * resolution + origin_y;
  return result;
}

// function for transform from PoseStamped to idx
int ToIndex(const geometry_msgs::PoseStamped& cord) {
  double resolution = grid_map_obj.info.resolution;
  double origin_x = grid_map_obj.info.origin.position.x;
  double origin_y = grid_map_obj.info.origin.position.y;
  int result_x = (cord.pose.position.x - origin_x) / resolution;
  int result_y = (cord.pose.position.y - origin_y) / resolution;
  return result_x + result_y * width;
}

// function for transform from Odometry to idx
int ToIndex(const nav_msgs::Odometry& cord) {
  double resolution = grid_map_obj.info.resolution;
  double origin_x = grid_map_obj.info.origin.position.x;
  double origin_y = grid_map_obj.info.origin.position.y;
  int result_x = (cord.pose.pose.position.x - origin_x) / resolution;
  int result_y = (cord.pose.pose.position.y - origin_y) / resolution;
  return result_x + result_y * width;
}

// weight function
double Weight(const int& first, const int& second) {
  double result = sqrt(std::pow(ToPare(first).first - ToPare(second).first, 2) + std::pow(ToPare(first).second - ToPare(second).second, 2));
  return result;
}

// abs function
int ABS(int x) {
  if (x < 0) {
    return -x;
  }
  return x;
}

// heuristic function for A* algorithm
double Heuristic(const int& idx, std::string type = "Manhattan") {
  const double constant = 1;
  if (type == "Chebyshev") {
    return constant * std::max(ABS(ToPare(idx).first - ToPare(ToIndex(goal)).first), ABS(ToPare(idx).second - ToPare(ToIndex(goal)).second));
  }
  if (type == "Manhattan") {
    return constant * ABS(ToPare(idx).first - ToPare(ToIndex(goal)).first) + ABS(ToPare(idx).second - ToPare(ToIndex(goal)).second);
  }
  return constant * Weight(idx, ToIndex(goal));
}

// A* priority queue comparator struct
struct Compare {
  bool operator()(const std::pair<int, double>& left, const std::pair<int, double>& right) {
    return (left.second > right.second);
  }
};

// upgrade of a map for dist
class MapDist {
 private:
  std::map<int, int> map;
  int kInf = 10000000;

 public:
  MapDist() = default;
  int& operator[](const int& key) {
    if (map.find(key) == map.end()) {
      map[key] = kInf;
    }
    return map[key];
  }
};

// upgrade of a map for parent
class MapParent {
 private:
  std::map<int, int> map;
  int default_parent = -1;

 public:
  MapParent() = default;
  int& operator[](const int& key) {
    if (map.find(key) == map.end()) {
      map[key] = default_parent;
    }
    return map[key];
  }
};

// main logic function, activates after definition of the goal point
void ListenerLogic(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // updating goal point position
  goal = *msg;
  std::cout << "GRID SIZE: " << grid_map.size() << '\n';
  std::cout << "GET GOAL: ";
  std::cout << "X: " << goal.pose.position.x << ' ';
  std::cout << "Y: " << goal.pose.position.y << ' ';
  std::cout << "Z: " << goal.pose.position.z << '\n';
  std::cout << "GET ODOM: ";
  std::cout << "X: " << odom.pose.pose.position.x << ' ';
  std::cout << "Y: " << odom.pose.pose.position.y << ' ';
  std::cout << "Z: " << odom.pose.pose.position.z << '\n';

  // logic constant for going out of map
  const bool out_box = true;
  // maximum difference of one step vision
  const int delta_step = 1;
  // starting time count
  ros::Time start_time = ros::Time::now();
  // define infinity of weights
  MapDist dist;
  MapParent parent;
  std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, Compare> que;
  dist[ToIndex(odom)] = 0;
  que.push(std::pair<int, double>(ToIndex(odom), Heuristic(ToIndex(odom))));
  parent[ToIndex(odom)] = ToIndex(odom);
  std::cout << "Start process...\n";
  int count = 0;
  bool miss_goal = true;
  // A* work cycle
  while (!que.empty() && miss_goal) {
    ++count;
    int u = que.top().first;
    que.pop();
    // checking goal point touch
    if (u == ToIndex(goal)) {
      miss_goal = false;
      break;
    }
    for (int dx = -delta_step; dx <= delta_step && miss_goal; ++dx) {
      for (int dy = -delta_step; dy <= delta_step && miss_goal; ++dy) {
        int v = ToIndex(ToPare(u).first + dx, ToPare(u).second + dy);
        if (v >= 0 && v < width * height && grid_map[v] < 100 && (grid_map[v] > -1 || out_box)) {
          double weight = Weight(u, v);
          if (dist[v] > dist[u] + weight) {
            dist[v] = dist[u] + weight;
            que.push(std::pair<int, double>(v, Heuristic(v) + dist[v]));
            parent[v] = u;
          }
        }
      }
    }
  }
  // getting goal point checking
  if (!miss_goal) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    std::vector<geometry_msgs::PoseStamped> reverse_path;
    for (int v = ToIndex(goal); v != ToIndex(odom); v = parent[v]) {
      // creating dot for the path
      geometry_msgs::PoseStamped dot;
      dot.pose.position.x = ToXY(v).first;
      std::cout << ToXY(v).first << ' ';
      dot.pose.position.y = ToXY(v).second;
      std::cout  << ToXY(v).second << '\n';
      // dot.pose.orientation.w = 1.0;
      reverse_path.emplace_back(dot);
    }
    for (int i = static_cast<int>(reverse_path.size()) - 1; i >= 0; --i) {
      path.poses.push_back(reverse_path[i]);
    }
    path_pub.publish(path);
    std::cout << "VAR: GOAL\n";
  } else {
    std::cout << "VAR: NO GOAL\n";
  }
  // ending time count
  ros::Time end_time = ros::Time::now();
  std_msgs::Float64 time_dif;
  time_dif.data = (end_time - start_time).toSec();
  std::cout << "TIME DIF: " << time_dif.data << '\n';
  // publishing time into time topic
  time_pub.publish(time_dif);
}

// function for updating odometry coordinates
void OdomUpdater(const nav_msgs::Odometry::ConstPtr& msg) {
  odom = *msg;
  /*
  std::cout << "GET ODOM: ";
  std::cout << "X: " << odom.pose.pose.position.x << ' ';
  std::cout << "Y: " << odom.pose.pose.position.y << ' ';
  std::cout << "Z: " << odom.pose.pose.position.z << '\n';
  */
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  path_pub = n.advertise<nav_msgs::Path>("path", 10);
  time_pub = n.advertise<std_msgs::Float64>("planning_time", 10);
  ros::Subscriber sub_map_updater = n.subscribe("map", 1000, MapUpdater);
  ros::Subscriber sub_logic = n.subscribe("move_base_simple/goal", 1000, ListenerLogic);
  ros::Subscriber sub_odometry = n.subscribe("odom", 1000, OdomUpdater);
  ros::spin();
  std::cout << "The end\n";
  return 0;
}
