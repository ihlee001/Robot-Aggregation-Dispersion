#include <libplayerc++/playerc++.h>
#include <iostream>
#include "communicate.h"
#include "args.h"
#include <string>
#include <time.h>
#include <sstream>
#include <stdlib.h>
#include <typeinfo>
#include <vector>
#include <iterator>
#include <math.h>

#define RAYS 32

//Project 3 for Iain Lee
using namespace PlayerCc;
using namespace std;

//Holds x and y coordinates as one object
struct Coords{
	double x;
	double y;
};

//Globals
string run_type;
double inter_dist;//distance but changed to inter_dist due to namespace
double d_sense;
int num_bots = 6;

/**************************************Checkers*****************************************/
//Checks to see if robot is in distance of sensors
bool is_within_dist(double robot_x, double robot_y, double x, double y, double dist){
	return sqrt((robot_x - x) * (robot_x - x) + (robot_y - y) * (robot_y - y)) <= dist;
}

//Checks to see if in max range of inter-robot distance with all robots in sense range
bool within_inter_dist(double x_pos[], double y_pos[], int id){
	bool within_inter = true;
	for(int i = 0; i < num_bots; i++){
		if(is_within_dist(x_pos[id], y_pos[id], x_pos[i], y_pos[i], d_sense)){
			if(!is_within_dist(x_pos[id], y_pos[id], x_pos[i], y_pos[i], inter_dist)){
				within_inter = false;
			}
		}
	}
	return within_inter;
}

//Checks to see if out of min range of inter-robot distance with all robots in sensors
bool out_inter_dist(double x_pos[], double y_pos[], int id){
	bool out_inter = true;
	for(int i = 0; i < num_bots; i++){
		if(is_within_dist(x_pos[id], y_pos[id], x_pos[i], y_pos[i], d_sense)){
			if(is_within_dist(x_pos[id], y_pos[id], x_pos[i], y_pos[i], inter_dist)){
				out_inter = false;
			}
		}
	}
	return out_inter;
}

/*************************************Getters*******************************************/
//Finds the centroid of the robots given a certain distance of sensing
Coords get_cent(double x_pos[], double y_pos[], int id){
	Coords centroid;
	double robot_x = x_pos[id];
	double robot_y = y_pos[id];
	int num_points = 0;
	centroid.x = 0;
	centroid.y = 0;
	for(int i = 0; i < num_bots; i++){
		if(is_within_dist(robot_x, robot_y, x_pos[i], y_pos[i], d_sense)){
			centroid.x += x_pos[i];
			centroid.y += y_pos[i];
			num_points++;
		}
	}
	centroid.x /= num_points;
	centroid.y /= num_points;
	return centroid;
}

//Finds the next direction to go to for dispersion
Coords get_dir(Coords centroid, double x_pos, double y_pos){
	Coords coords;
	double dx = x_pos - centroid.x;
	double dy = y_pos - centroid.y;

	coords.x = x_pos + 2 * dx;
	coords.y = y_pos + 2 * dy;
	return coords;
}

/**********************************Motor Functions***************************************/
//Motor Function that allows for dispersion of the robots
void dispr(PlayerClient &robot, Position2dProxy &pp, LaserProxy &lp, double x_pos[], 
			double y_pos[], int id){
	robot.Read();
	x_pos[id] = pp.GetXPos();
	y_pos[id] = pp.GetYPos();

	if(!out_inter_dist(x_pos, y_pos, id)){//not max inter-robot distance 
		double right = lp.GetMinRight();
		double left = lp.GetMinLeft();
		if(right < 1 || left < 1){
			if(right < left) pp.SetSpeed(0.1, 2 * M_PI / 9);//turn left if right block
			else pp.SetSpeed(0.1, -2 * M_PI / 9);//turn right if left block
		}
		else{
			Coords coords = get_dir(get_cent(x_pos, y_pos, id), x_pos[id], y_pos[id]);
			pp.GoTo(coords.x, coords.y, 0);	
		}
	}
	else pp.SetSpeed(0, 0);
}

//Motor Function that allows for aggregation of the robots and tries to avoid collision
void aggr(PlayerClient &robot, Position2dProxy &pp, LaserProxy &lp, double x_pos[], 
			double y_pos[], int id){
	robot.Read();
	x_pos[id] = pp.GetXPos();
	y_pos[id] = pp.GetYPos();
	
	if(!within_inter_dist(x_pos, y_pos, id)){//if not within inter-robot distance
		//avoidance of collision if a robot is right in front stop
		if(lp.GetMinRight() < .65 || lp.GetMinLeft() < .65) pp.SetSpeed(0, 0);
		else{
			Coords centroid = get_cent(x_pos, y_pos, id);
			pp.GoTo(centroid.x, centroid.y, 0);		
		}
	}
	else pp.SetSpeed(0, 0);
}

/*************************************Location******************************************/
//Parses the message and places them into the correct slot of the x and y position arrays
void update_location(string message, double x_pos[], double y_pos[]){
	vector<string> split_message;
	istringstream splitter(message);
	istream_iterator<string> beg(splitter), end;
	
	vector<string> tokens(beg, end);
	int id = atoi(tokens[0].c_str());
	x_pos[id] = atof(tokens[1].c_str());
	y_pos[id] = atof(tokens[2].c_str());
}

//Broadcast the location of the robot
void broadcast_location(PlayerClient &robot, Position2dProxy &pp, int id, int b_fd){
	stringstream ss;
	robot.Read();
	ss << id << " " << pp.GetXPos() << " " << pp.GetYPos();
	talk_to_all(b_fd, const_cast<char*>(ss.str().c_str()), H);
}

/**********************************Robot Creation***************************************/
//starts the robots and assigns them into the arrays
void start_robot(int port, PlayerClient** &robots, Position2dProxy** &ppp, 
					LaserProxy** &lpp, int i){
	robots[i] = new PlayerClient(gHostname, port);
	ppp[i] = new Position2dProxy(robots[i], gIndex);
	lpp[i] = new LaserProxy(robots[i], gIndex);
}

//Function that creates the robots and runs them one at a time 
void create_robots(int broadcast_fd, int listen_fd){
	double x_pos[num_bots];//array of x positions of the robots
	double y_pos[num_bots];//array of y positions of the robots
	int nbytes;
	char msg[MAXBUF];
	
	PlayerClient** robots = new PlayerClient*[num_bots];//Pointer to array of robots
	Position2dProxy** ppp = new Position2dProxy*[num_bots];//Pointer to array of proxies
	LaserProxy** lpp = new LaserProxy*[num_bots];//Pointer to array of lasers

	for(int i = 0; i < num_bots; i++){
		start_robot(6665 + i, robots, ppp, lpp, i);//Starts the robots
		ppp[i] -> SetMotorEnable(true);//Enables their motors
		broadcast_location(*robots[i], *ppp[i], i, broadcast_fd);
		nbytes = listen_to_robot(listen_fd, msg);
		if(nbytes != 0) update_location(string(msg), x_pos, y_pos);
	}

	//every robot reports first and then listens to update locations of positions
	while(true){
		for(int i = 0; i < num_bots; i++){
			broadcast_location(*robots[i], *ppp[i], i, broadcast_fd);
			nbytes = listen_to_robot(listen_fd, msg);
			if(nbytes != 0){ 
				update_location(string(msg), x_pos, y_pos);
				//if a for aggregation or d for dispersion
				if(run_type == "a") aggr(*robots[i], *ppp[i], *lpp[i], x_pos, y_pos, i);
				else if(run_type == "d"){
					dispr(*robots[i], *ppp[i], *lpp[i], x_pos, y_pos, i);
				}
			}
		}
	}

}

/***************************************Main********************************************/
int main(int argc, char **argv){
	if(argc < 4){
		cout << "project3 run_type d_sense distance\n";
	}
	run_type = string(argv[1]);//assigns globals to passed arguments
	d_sense = atof(argv[2]) * 0.0254;
	inter_dist = atof(argv[3]) * 0.0254;

	int broadcast_fd = create_broadcast(PORT_H, H);//Creating broadcast port
	int listen_fd = create_listen(PORT_H, H);//Creating listening port
	
	create_robots(broadcast_fd, listen_fd);
}
