#include "shared_constants.h"
#include <stdlib.h>
#include <stdio.h>
#include <robot_if++.h>
#include <unistd.h>
#include "RobotPose.cpp"
#include "CameraPose.cpp"
#include <iostream>
#include <string>
#define R1 0
#define R2 1


void getMap();
void get_moves(path *r1, path *r2);
void search_paths(path *r1, vector<int> *path_x, vector<int> *path_y, int value, int bonus, int depth, int max_depth, int pos_x, int pos_y, int r2x, int r2y);
void try_move(path *r1, path *r2);
void move(path *r1, path *r2);
int valid_move(path *r1, path *r2);

int moves_left = 27;
int r1_score = 0;
int r2_score = 0;

//The true score of the game from the server
int score1=0, score2=0;

int path_found=0;

path **paths;
path *r1;
path *r2;

int maze[5][7];
int maze_visited[5][7];
RobotPose *robotPose;
RobotInterface *robot;
map_obj_t *robot_map;


int main(int argv, char **argc) {
	paths = new path*[2];
	paths[0] = (path*)malloc(sizeof(path));
	paths[0]->moves_x = new vector<int>();
	paths[0]->moves_y = new vector<int>();
	
	paths[1] = (path*)malloc(sizeof(path));
	paths[1]->moves_x = new vector<int>();
	paths[1]->moves_y = new vector<int>();
    
	paths[0]->curr_x = 0;
	paths[0]->curr_y = 2;
    
	paths[1]->curr_x = 6;
	paths[1]->curr_y = 2;

// Make sure we have a valid command line argument
	if(argv <= 2) {
		std::cout << "Usage: robot_test <address of robot> " << std::endl;
		exit(-1);
	}
    
	if(0==strncmp(argc[1],"rosie",strlen("rosie"))){
	      ns_x_to_cm = rosie_ns_x_to_cm;
	      ns_y_to_cm = rosie_ns_x_to_cm;
	      we_to_cm = rosie_we_to_cm;
	      ns_theta_offsets = rosie_ns_theta_offsets;
	      
	      // If rosies's WE are bad set them to NS???
	      
	}else if(0==strncmp(argc[1],"bender",strlen("bender"))){
	      ns_x_to_cm = bender_ns_x_to_cm;
	      ns_y_to_cm = bender_ns_x_to_cm;
	      we_to_cm = bender_we_to_cm;
	      ns_theta_offsets = bender_ns_theta_offsets;
	}else{
	      printf("Bot Not supported!\n"); 
	      exit(-1);
	}
    
	// Setup the robot
	
	printf("%s %d\n", argc[1], atoi(argc[2]));
	
	robot = new RobotInterface(argc[1],atoi(argc[2]));
	robotPose = new RobotPose(robot, argc[2]);
	
	robot->update();
	getMap();
	
	while (moves_left > 0) {
		if(robotPose->player == 1)
			move(paths[0], paths[1]);
		else if(robotPose->player == 2)
			move(paths[1], paths[0]);
	}
}

void get_moves(path *r1, path *r2) {
	int depth = 3;	
	vector<int> *path_x = new vector<int>();
	vector<int> *path_y = new vector<int>();
	int pos_x = r1->curr_x;
	int pos_y = r1->curr_y;
	printf("(%d, %d)\n",pos_x, pos_y);
    
	//r1->moves_x->clear();
	//r1->moves_y->clear();
	r1->value = 0;
	// come up with a sequence of moves and put them into r1->moves_x and r1->moves_y
	

	
	while (r1->value == 0&& depth<35) {
	    for (int i = 0; i < 5; i++) {
		for (int j = 0; j < 7; j++) {
		    maze_visited[i][j] = 1;
		}
	    }
	    
	    search_paths(r1, path_x, path_y, 0, 0, 0, depth++, pos_x, pos_y, r2->curr_x, r2->curr_y);
	    
	    
	    printf("Paths: %d\n",r1->moves_x->size());
	    for(int k=0;k<r1->moves_x->size();k++){
	      int x = r1->moves_x->at(k);
	      int y = r1->moves_y->at(k);
	     printf("(%d %d) -> ",x, y); 
	    }
	    
	    printf("%d \n", r1->value);
	    
	    for(int r=0;r<5;r++){
		for(int c=0;c<7;c++){
		  printf("%d\t",maze[r][c]);
		}
		printf("\n");
	    }
	    printf("\n");
	    printf("\n");
	    printf("\n");
	    
	//exit(0);
	    
	}
}

void search_paths(path *r1, vector<int> *path_x, vector<int> *path_y,
		  int value, int bonus, int depth,
		  int max_depth, int pos_x, int pos_y, int r2x, int r2y) {
    if (pos_x >= 0 && pos_y >= 0 && pos_x < 7 && pos_y < 5 
	    && maze[pos_y][pos_x] >= 0 
	    /*&& !(pos_x == r2x && pos_y == r2y) */
	    && maze_visited[pos_y][pos_x]) {
	
	maze_visited[pos_y][pos_x] = 0;
	int last = path_x->size() - 1;
	if (last >= 2) {
	    int delta_x1 = path_x->at(last) - path_x->at(last - 1);
	    int delta_x2 = path_x->at(last - 1) - path_x->at(last - 2);
	    int delta_y1 = path_y->at(last) - path_y->at(last - 1);
	    int delta_y2 = path_y->at(last - 1) - path_y->at(last - 2);
	    
	    if (delta_x1 == delta_x2 && delta_y1 == delta_y2) {
		bonus += 2;
	    }
	}
    
	value += maze[pos_y][pos_x] + bonus;
	depth++;
    
	if (depth >= max_depth) {
	    if (value > 0 && value + bonus > r1->value) {
		r1->moves_x->clear();
		r1->moves_y->clear();
		r1->value = value + bonus;
		
		for (int i = 0; i < path_x->size(); i++) {
		    r1->moves_x->push_back(path_x->at(i));
		    r1->moves_y->push_back(path_y->at(i));
		}
	    }
	}
	else {
	 
	    path_x->push_back(pos_x + 1);
	    path_y->push_back(pos_y);
	    search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
	    path_x->pop_back();
	    //path_y->pop_back();
	    
	    path_x->push_back(pos_x - 1);
	    //path_y->push_back(pos_y);
	    search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
	    path_x->pop_back();
	    path_y->pop_back();
	    
	    path_x->push_back(pos_x);
	    path_y->push_back(pos_y + 1);
	    search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
	    //path_x->pop_back();
	    path_y->pop_back();
	    
	    //path_x->push_back(pos_x);
	    path_y->push_back(pos_y - 1);
	    search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
	    path_x->pop_back();
	    path_y->pop_back();
	}
    }
}

void try_move(path *r1, path *r2) {
	if (r1->moves_x->size() == 0) { // if there are no moves left, generate more
		get_moves(r1, r2);
		try_move(r1, r2);
	}
	else {
		move(r1, r2); // try to make a move
	}
}

void move(path *r1, path *r2) {
    int ret = -1;
    if((r1->moves_x->size()>0 && !(r1->moves_x->front() == r2->curr_x))){
      ret = robot->reserveMap(r1->moves_x->front(), r1->moves_y->front());
      printf("Ret: %d: %d %d \n",ret,r1->moves_x->front(),r1->moves_y->front());
    }
	if (0==ret) {
		r1->curr_x = r1->moves_x->front();
		r1->curr_y = r1->moves_y->front();
		// reserve r1->curr_x and r1->curr_y 
		robotPose->moveToCell(r1->curr_x, r1->curr_y);
		r1->moves_x->erase(r1->moves_x->begin());
		r1->moves_y->erase(r1->moves_y->begin());
    
		//sleep(2);
		if (maze[r1->curr_y][r1->curr_x] > 0) {
			moves_left--;
			r1_score += maze[r1->curr_y][r1->curr_x];
			//maze[r1->curr_y][r1->curr_x] = 0;
			getMap();
		}
		
		// moveToCell goes here
		robot->updateMap(r1->curr_x, r1->curr_y);
		
		
	}
	else { // give up and try generating new moves
		r1->moves_x->clear();
		r1->moves_y->clear();
		try_move(r1, r2);
	}
}

int valid_move(path *r1, path *r2) {
	return (r1->moves_x->size()>0 && !(r1->moves_x->front() == r2->curr_x && r1->moves_y->front() == r2->curr_y));
}

void getMap(){
  assert(NULL!= robot);
	map_obj_t * mapList = robot->getMap(&score1, &score2);
	if(mapList == NULL){
		printf("Error getting map\n");
	}
	int robot1 = 0;
	int robot2 = 0;
	if(robotPose->player == 1){
		robot2 = -1;
	}
	else if(robotPose->player == 2){
		robot1 = -1;
	}
	for(int y=0; y<5; y++){
		for(int x=0; x<7; x++){
		  
			if(MAP_OBJ_POST == (int)mapList->type){
			  maze[y][x] = -1;
			}else if(MAP_OBJ_ROBOT_1  == (int)mapList->type){
			  maze[y][x] = robot1;
			}else if(MAP_OBJ_ROBOT_2 == (int)mapList->type){
			  maze[y][x] = robot2;
			}else if(MAP_OBJ_PELLET == (int)mapList->type){
			  maze[y][x] = mapList->points;	
			}else if(MAP_OBJ_EMPTY == (int)mapList->type){
			  maze[y][x] = mapList->points;	
			}else if(MAP_OBJ_RESERVE_1 == (int)mapList->type){
			  maze[y][x] = robot1;	
			}else if(MAP_OBJ_RESERVE_2 == (int)mapList->type){
			  maze[y][x] = robot2;	
			}else{
			 maze[y][x] = -999; 
			}
			mapList = mapList->next;
		}
	}
	
}