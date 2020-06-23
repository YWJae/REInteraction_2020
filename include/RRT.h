#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>

#define WAYPOINT_NUM_MAX 10000


struct Obstacle {
    std::string type;

    double center_x;
	double center_y;
    double radius;
};

struct Point {
    double x;
    double y;
    double proximity;
};


struct Node {
    double x;
    double y;
    double proximity;
    int id;
    int parent;
};


class RRT {
    private:
		int rrt_unique_id_ = 1;
		int rrt_max_iteration_;
		int rrt_epsilon_;
		double rrt_step_size_;
		double rrt_threshold_;

		double bound_x_min_;
		double bound_x_max_;
		double bound_y_min_;
		double bound_y_max_;

        double start_position_x_;
        double start_position_y_;

		double target_position_x_;
		double target_position_y_;

		double obstacle_1_[3];
		double obstacle_2_[3];
		double obstacle_3_[3];

		int remainder_;

    public:
		int sampled_size_;
		int waypoint_size_;
		double sampled_[WAYPOINT_NUM_MAX][2]; // down sampled
		double waypoint_[WAYPOINT_NUM_MAX][2]; // raw data (trajectory)

        RRT(double* start,  double* goal, double thres, double* obstacle, double* x_bounds, double* y_bounds, double step_size, double eps, double iter);
        Point newPoint(std::uniform_real_distribution<double> x_unif, std::uniform_real_distribution<double> y_unif, std::default_random_engine& re);
        Node closestNode(std::vector<Node>& tree, Point& this_point);
        Node takeStep(Point rand_point, Node nearest_node, double step_size, int& uniq_id);
        void generateObstacles(std::vector<Obstacle>& obstacles);
		static bool sortByDist(const Node &node_1, const Node &node_2) { return node_1.proximity < node_2.proximity; };
        bool checkPointCollision(std::vector<Obstacle>& obstacles, Point& possible_point);
		bool checkNodeCollision(std::vector<Obstacle>& obstacles, Node& possible_node);
        bool foundGoal(Point goal_pt, Node node, double THRESHOLD);
        Node getNodefromID(int id, std::vector<Node> Nodes);
        void getPath(std::vector<Node> Tree, Node last_node);
        void solve();
};

