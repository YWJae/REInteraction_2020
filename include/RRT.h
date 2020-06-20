#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>

#define WAYPOINT_NUM_MAX 1000


struct Obstacle {
    std::string type;

 
    double center_x[3];
	double center_y[3];
    double radius[3];
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
        double START_X;
        double START_Y;

		
		double POINT1_X;
		double POINT1_Y;

		double OBSTACLE_X1;
		double OBSTACLE_Y1;
		double OBSTACLE_R1;
		double OBSTACLE_X2;
		double OBSTACLE_Y2;
		double OBSTACLE_R2;
		double OBSTACLE_X3;
		double OBSTACLE_Y3;
		double OBSTACLE_R3;

        double GOAL_X;
        double GOAL_Y;
        double THRESHOLD;

        double X_MIN;
        double X_MAX;
        double Y_MIN;
        double Y_MAX;

        double STEP_SIZE;
        int EPSILON;

        int unique_id = 1;
        int MAX_ITER;

    public:
		int remainder;
		double sampled_[WAYPOINT_NUM_MAX][2];
		double waypoint_[WAYPOINT_NUM_MAX][2];
		int sampled_size_;
		int waypoint_size_;
        RRT(double* start,  double* goal, double thres, double* obstacle, double* x_bounds, double* y_bounds, double step_size, double eps, double iter);
        Point newPoint(std::uniform_real_distribution<double> x_unif, std::uniform_real_distribution<double> y_unif, std::default_random_engine& re);
        static bool sortByDist(const Node &node_1, const Node &node_2) { return node_1.proximity < node_2.proximity;};
        Node closestNode(std::vector<Node>& tree, Point& this_point);
        Node takeStep(Point rand_point, Node nearest_node, double step_size, int& uniq_id);
        void generateObstacles(std::vector<Obstacle>& obstacles);
        bool checkPointCollision(std::vector<Obstacle>& obstacles, Point& possible_point);
		bool checkNodeCollision(std::vector<Obstacle>& obstacles, Node& possible_node);
        bool foundGoal(Point goal_pt, Node node, double THRESHOLD);
        Node getNodefromID(int id, std::vector<Node> Nodes);
        void getPath(std::vector<Node> Tree, Node last_node);
        void solve();
};

