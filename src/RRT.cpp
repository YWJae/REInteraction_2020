#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "RRT.h"

//using namespace std;
std::ofstream logfile2;


RRT::RRT(double* start, double* goal, double thres, double* obstacle, double* x_bounds, double* y_bounds, double step_size, double eps, double iter){
    START_X = start[0];
    START_Y = start[1];
	
	
	GOAL_X = goal[0];
    GOAL_Y = goal[1];

    THRESHOLD = thres;


	OBSTACLE_X1 = obstacle[0];//[0];
	OBSTACLE_Y1 = obstacle[1];//[1];
	OBSTACLE_R1 = obstacle[2];//[2];
	OBSTACLE_X2 = obstacle[3];//[0];
	OBSTACLE_Y2 = obstacle[4];//[1];
	OBSTACLE_R2 = obstacle[5];//[2];
	OBSTACLE_X3 = obstacle[6];//[0];
	OBSTACLE_Y3 = obstacle[7];//[1];
	OBSTACLE_R3 = obstacle[8];//[2];

    X_MIN = x_bounds[0];
    X_MAX = x_bounds[1];

    Y_MIN = y_bounds[0];
    Y_MAX = y_bounds[1];

    STEP_SIZE = step_size;
    EPSILON = eps;
    MAX_ITER = iter;
};

Point RRT::newPoint(std::uniform_real_distribution<double> x_unif, std::uniform_real_distribution<double> y_unif, std::default_random_engine& re) {
    double x = x_unif(re);
    double y = y_unif(re);
    
    Point new_point;
    new_point.x = x;
    new_point.y = y;

    return new_point;

};

Node RRT::closestNode(std::vector<Node>& tree, Point& this_point) {
    for(int i=0; i<tree.size();i++) {
        double delt_x_sqrd = pow((tree[i].x - this_point.x), 2);
        double delt_y_sqrd = pow((tree[i].y - this_point.y), 2);
        double dist = sqrt(delt_x_sqrd + delt_y_sqrd);
        tree[i].proximity = dist;
    };

    sort(tree.begin(), tree.end(), sortByDist);

    return tree[0];
};

Node RRT::takeStep(Point rand_point, Node nearest_node, double step_size, int& uniq_id) {
    double dir_x = rand_point.x - nearest_node.x;
    double dir_y = rand_point.y - nearest_node.y;
    double dir_x_sqrd = pow(dir_x, 2);
    double dir_y_sqrd = pow(dir_y, 2);
    double magnitude = sqrt(dir_x_sqrd + dir_y_sqrd);

    double unit_x = dir_x/magnitude;
    double unit_y = dir_y/magnitude;

    double step_x = step_size*unit_x;
    double step_y = step_size*unit_y;

    Node new_node;
    new_node.x = nearest_node.x+step_x;
    new_node.y = nearest_node.y+step_y;
    new_node.id = uniq_id;
    new_node.parent = nearest_node.id;

    uniq_id++;

    return new_node;
};

void RRT::generateObstacles(std::vector<Obstacle>& obstacles) {
	
	Obstacle obj;

	obj.center_x[0] =OBSTACLE_X1   ;
	obj.center_y[0] =OBSTACLE_Y1   ;
	obj.radius[0]   =OBSTACLE_R1   ;
	obj.center_x[1] =OBSTACLE_X2   ;
	obj.center_y[1] =OBSTACLE_Y2   ;
	obj.radius[1]   =OBSTACLE_R2   ;
	obj.center_x[2] =OBSTACLE_X3   ;
	obj.center_y[2] =OBSTACLE_Y3   ;
	obj.radius[2]   =OBSTACLE_R3   ;

	obstacles.push_back(obj);
	//Obstacle obstacle1;
    //
	//obstacle1.center_x[0] = 3.;
	//obstacle1.center_y[0] = 2.;
	//obstacle1.radius[0] = .5;
	//obstacles.push_back(obstacle1);
	//
	//Obstacle obstacle2;
	//
	//obstacle2.center_x[1] = 5.;
	//obstacle2.center_y[1] = 1.;
	//obstacle2.radius[1] = 1.;
	//obstacles.push_back(obstacle2);
	//Obstacle obstacle3;
	//
	//obstacle3.center_x[2] = 5.;
	//obstacle3.center_y[2] = 3.;
	//obstacle3.radius[2] = .5;
	//obstacles.push_back(obstacle3);
	   
};

bool RRT::checkPointCollision(std::vector<Obstacle>& obstacles, Point& possible_point) {
	for (int j = 0; j<3; j++) {
		double dx2 = pow((obstacles[0].center_x[j] - possible_point.x), 2);
		double dy2 = pow((obstacles[0].center_y[j] - possible_point.y), 2);
		bool dist_obstacle = sqrt(dx2 + dy2) < obstacles[0].radius[j]; // Obstacle과 충돌 계산
		//std::cout << dist_obstacle << std::endl;
		//std::cout << "Possible point: " << possible_point.x << "\t" << possible_point.y << std::endl;
		//std::cout << "R: " << sqrt(dx2 + dy2) << "\t" << obstacles[0].radius[j] << std::endl;
		bool out_of_bound = abs(possible_point.x) < abs(X_MAX) && abs(possible_point.y) < abs(Y_MAX); // 경기장 bound 충돌 계산
		if (dist_obstacle && out_of_bound) {
			//std::cout << "YYYYYYYYYYY" << std::endl;
			return true;
		};
	};
	//std::cout << "CCCCCCCCCCCCCCCC" << std::endl;
	return false;
};

bool RRT::checkNodeCollision(std::vector<Obstacle>& obstacles, Node& possible_node) {

	for (int j = 0; j<3; j++) {
		double dx2 = pow((obstacles[0].center_x[j] - possible_node.x), 2);
		double dy2 = pow((obstacles[0].center_y[j] - possible_node.y), 2);
		bool dist_obstacle = sqrt(dx2 + dy2) < obstacles[0].radius[j]; // Obstacle과 충돌 계산
		bool out_of_bound = abs(possible_node.x) < abs(X_MAX) && abs(possible_node.y) < abs(Y_MAX); // 경기장 bound 충돌 계산
		if (dist_obstacle && out_of_bound) {
			return true;
		};
	};
	return false;
};

bool RRT::foundGoal(Point goal_pt, Node node, double THRESHOLD) {
    double x_dist_sqrd = pow((goal_pt.x-node.x), 2);
    double y_dist_sqrd = pow((goal_pt.y-node.y), 2);

    double dist = sqrt(x_dist_sqrd+y_dist_sqrd);

    if (dist<THRESHOLD) {
        return true;
    };

    return false;
};

Node RRT::getNodefromID(int id, std::vector<Node> Nodes) {
    for (int i=0; i<Nodes.size(); i++) {
        if (Nodes[i].id == id) {
            return Nodes[i];
        };
    };
};

void RRT::getPath(std::vector<Node> Tree, Node last_node) {
    std::vector<Node> path;
    path.push_back(last_node);
    Node current_node = last_node;
    Node parent_node;

    int counter = 0;
    while (current_node.id != 0) {
        int parent_id = current_node.parent;
        parent_node = getNodefromID(parent_id, Tree);
        path.push_back(parent_node);
        current_node = parent_node;
    };

	std::reverse(path.begin(), path.end());

	waypoint_size_ = path.size();
	sampled_size_ = int(path.size() / 3);

	for (int i = 0; i < path.size(); i++) {
		waypoint_[i][0] = path[i].x;
		waypoint_[i][1] = path[i].y;
	}

	int count = 0;
	for (int i = 0; i < path.size(); i++) {
		int idx = i / 3;
		remainder = i % 3;
		//std::cout << "the remainder is" << remainder << std::endl;
		waypoint_[i][0] = path[i].x;
		waypoint_[i][1] = path[i].y;

		if (remainder == 0)
		{
			sampled_[count][0] = path[i].x;
			sampled_[count][1] = path[i].y;
			//std::cout << "/////////////////////" << std::endl;
			//std::cout << "count" << count << std::endl;
			//std::cout << "path x" << path[i].x << std::endl;
			//std::cout << "path y" << path[i].y << std::endl;
			//std::cout << "sampled_x" << sampled_[count][0] << std::endl;
			//std::cout << "sampled_y" << sampled_[count][1] << std::endl;
			count += 1;
		}
	}

	
	//for (int m = 0; m<path.size(); m++) {
	//	std::cout << waypoint_[m][0] << std::endl;
	//};
	//std::cout << std::endl << std::endl;
	//for (int m = 0; m<path.size(); m++) {
	//	std::cout << waypoint_[m][1] << std::endl;
	//};

};

void RRT::solve() {
    std::uniform_real_distribution<double> x_dist(X_MIN, X_MAX);
    std::uniform_real_distribution<double> y_dist(Y_MIN, Y_MAX);

    std::default_random_engine re;

    std::vector<Node> Tree;
    std::vector<Obstacle> Obstacles;

    generateObstacles(Obstacles);

    Point goal_point;
    goal_point.x = GOAL_X;
    goal_point.y = GOAL_Y;

    Node init_node;
    init_node.x = START_X;
    init_node.y = START_Y;
    init_node.id = 0;
    init_node.parent = 0;

    Tree.push_back(init_node);
    
    int i = 0;

    while (i<MAX_ITER) {
        Point rand_pt = newPoint(x_dist, y_dist, re);
		
        if (i % EPSILON == 0 && i != 0) {
            rand_pt.x = GOAL_X;
            rand_pt.y = GOAL_Y;
        }

        if (checkPointCollision(Obstacles, rand_pt)) {
            while (checkPointCollision(Obstacles, rand_pt)) {
                rand_pt = newPoint(x_dist, y_dist, re);
            };
        };

        Node closest_node = closestNode(Tree, rand_pt);
        Node new_node = takeStep(rand_pt, closest_node, STEP_SIZE, unique_id);

        if (checkNodeCollision(Obstacles, new_node)) {
            i++;
            continue;
            };

        Tree.push_back(new_node);

        if (foundGoal(goal_point, new_node, THRESHOLD)) {
            std::cout<<"*******GOAL FOUND******"<<std::endl;
            getPath(Tree, new_node);
            break;
        };

        i++;
    };
};

/*
int main () {
	double x_bounds[2] = { -0.100, 0.100 }; // 경기장 X bound
	double y_bounds[2] = { -0.200, 0.200 }; // 경기장 Y bound

	double start[2] = { 0.08, -0.18 };  // position 1
	double target1[2] = { -0.08,  0.18 };  // position 2
	double target2[2] = { 0.08,  0.18 };  // position 3
	double target3[2] = { -0.08, -0.18 };  // position 4

	double rrt_threshold = 0.01;
	double rrt_step_size = 0.01;
	int rrt_epsilon = 100;
	int rrt_max_iteration = 10000;


	double obstacle_[9];
	double padding_obstacle_(0.0);
	obstacle_[0] = 0.02;	obstacle_[1] = -0.03;	obstacle_[2] = 0.035 + padding_obstacle_;
	obstacle_[3] = -0.07;	obstacle_[4] = -0.15; 	obstacle_[5] = 0.05 + padding_obstacle_;
	obstacle_[6] = 0.04;	obstacle_[7] = -0.23;	obstacle_[8] = 0.025 + padding_obstacle_;


    RRT my_rrt(start, target1, rrt_threshold, obstacle_, x_bounds, y_bounds, rrt_step_size, rrt_epsilon, rrt_max_iteration);
    my_rrt.solve();

	int n1; n1 = my_rrt.waypoint_size_;
	logfile2.open("rrt1.txt");
	for (int i = 0; i < n1; i++) {
		logfile2 << my_rrt.waypoint_[i][0] << "\t" << my_rrt.waypoint_[i][1] << std::endl;
	}
	logfile2.close();
	std::cout << my_rrt.waypoint_size_ << std::endl;

	RRT my_rrt2(target1, target2, rrt_threshold, obstacle_, x_bounds, y_bounds, rrt_step_size, rrt_epsilon, rrt_max_iteration);
	my_rrt2.solve();

	int n2; n2 = my_rrt2.waypoint_size_;
	logfile2.open("rrt2.txt");
	for (int i = 0; i < n2; i++) {
		logfile2 << my_rrt2.waypoint_[i][0] << "\t" << my_rrt2.waypoint_[i][1] << std::endl;
	}
	logfile2.close();
	std::cout << my_rrt.waypoint_size_ << std::endl;

	RRT my_rrt3(target2, target3, rrt_threshold, obstacle_, x_bounds, y_bounds, rrt_step_size, rrt_epsilon, rrt_max_iteration);
	my_rrt3.solve();

	int n3; n3 = my_rrt3.waypoint_size_;
	logfile2.open("rrt3.txt");
	for (int i = 0; i < n3; i++) {
		logfile2 << my_rrt3.waypoint_[i][0] << "\t" << my_rrt3.waypoint_[i][1] << std::endl;
	}
	logfile2.close();
	std::cout << my_rrt3.waypoint_size_ << std::endl;
	getchar();

	//RRT my_rrt1(point1, point2, THRESHOLD, obstacle, x_bounds, y_bounds, STEP_SIZE, EPSILON, MAX_ITER);
	//my_rrt1.solve();
	//RRT my_rrt2(point2, end, THRESHOLD, obstacle, x_bounds, y_bounds, STEP_SIZE, EPSILON, MAX_ITER);
	//my_rrt2.solve();
	//std::cout<< x_dist, y_dist;
   return 0;
}
*/