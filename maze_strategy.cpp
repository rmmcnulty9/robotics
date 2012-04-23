/*
 * maze_strategy program
 *  Used to move robot around maze.
 *  Contains the pathfinding algorithm.
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */

#include "shared_constants.h"
#include <stdlib.h>
#include <stdio.h>
#include <robot_if++.h>
#include <unistd.h>
#include "RobotPose.cpp"
#include "CameraPose.cpp"
#include <iostream>
#include <string>
#include "maze_strategy.h"

/*
* Main function of the program. Works for both sides of the maze,
* so two instances of the program can compete against each other.
* */
int main(int argv, char **argc) {
    
	// Initialization
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
	if (argv <= 2) {
		std::cout << "Usage: maze_strategy <name of robot> <team of robot (1 or 2)>" << std::endl;
		exit(-1);
	}
    
	// Get the correct constants for our robot
	if (0 == strncmp(argc[1], "rosie", strlen("rosie"))) {
		ns_x_to_cm = rosie_ns_x_to_cm;
		ns_y_to_cm = rosie_ns_x_to_cm;
		we_to_cm = rosie_we_to_cm;
		ns_theta_offsets = rosie_ns_theta_offsets;
	      
	      // If rosies's WE are bad set them to NS???	      
	}
	else if (0 == strncmp(argc[1], "bender", strlen("bender"))) {
		ns_x_to_cm = bender_ns_x_to_cm;
		ns_y_to_cm = bender_ns_x_to_cm;
		we_to_cm = bender_we_to_cm;
		ns_theta_offsets = bender_ns_theta_offsets;	  
	}
	else if (0 == strncmp(argc[1], "walle", strlen("walle"))) {
		ns_x_to_cm = bender_ns_x_to_cm;
		ns_y_to_cm = bender_ns_x_to_cm;
		we_to_cm = bender_we_to_cm;
		ns_theta_offsets = bender_ns_theta_offsets;
	}
	else {
		printf("Bot Not supported!\n"); 
		exit(-1);
	}
    
	// Set up the robot	
	printf("%s %d\n", argc[1], atoi(argc[2]));
	
	robot = new RobotInterface(argc[1], atoi(argc[2]));
	robotPose = new RobotPose(robot, argc[2]);
	
	robot->update();
	getMap();
	
	while (moves_left > 0) {
		if (robotPose->player == 1) {
			move(paths[0], paths[1]);
		}
		else if (robotPose->player == 2) {
			move(paths[1], paths[0]);
		}
		
		getMap();
	}
}

/*
* Tries to find a valid path for r1 to follow. 
* Uses depth-first search with iterative deepening
* */
void get_moves(path *r1, path *r2) {
	int depth = 2;	
	vector<int> *path_x = new vector<int>();
	vector<int> *path_y = new vector<int>();
	printf("(%d, %d)\n", r1->curr_x, r1->curr_y);
    
	r1->moves_x->clear();
	r1->moves_y->clear();
	r1->value = 0;
	
	// come up with a sequence of moves and put them into r1->moves_x and r1->moves_y
	getMap(); // make sure the map data is fresh before we start searching
	while (r1->value == 0 && depth <= 35) {
	    for (int i = 0; i < 5; i++) {
			for (int j = 0; j < 7; j++) {
				maze_visited[i][j] = 1;
			}
	    }
	    
	    // Start searching for moves, depth increases with each iteration
		search_paths(r1, path_x, path_y, 0, 0, 0, depth++, r1->curr_x, r1->curr_y, r2->curr_x, r2->curr_y);
	    
		// Print path information
		printf("Path length: %d\n", r1->moves_x->size());
		printf("Depth: %d\n", depth - 1);
		for (int k = 0; k < r1->moves_x->size(); k++) {
			int x = r1->moves_x->at(k);
			int y = r1->moves_y->at(k);
			printf("(%d %d) -> ", x, y); 
		}
	    
		printf("%d \n", r1->value);
	    
		for (int r = 0; r < 5; r++) {
			for (int c = 0; c < 7; c++) {
				printf("%d\t", maze[r][c]);
			}
			printf("\n");
		}
		printf("\n");
		printf("\n");
		printf("\n");
		//exit(0);
	}
	
	// If we didn't find any valid path, give up
	if (r1->moves_x->size() == 0) {
		printf("No path was found. Exiting.\n");
		exit(0);
	}
}

/*
* Considers all paths of a given depth for r1 to follow (and picks the best one), 
* using recursive depth-first search
* */
void search_paths(path *r1, vector<int> *path_x, vector<int> *path_y,
	int value, int bonus, int depth,
	int max_depth, int pos_x, int pos_y, int r2x, int r2y) {
	
	if (pos_x >= 0 && pos_y >= 0 && pos_x < 7 && pos_y < 5 
		&& maze[pos_y][pos_x] >= 0 
		/*&& !(pos_x == r2x && pos_y == r2y) */
		&& maze_visited[pos_y][pos_x]) {
	
		maze_visited[pos_y][pos_x] = 0;
		int last = path_x->size() - 1;
		
		// Give a bonus for traveling in a straight line because it is faster
		if (last >= 2) {
			int delta_x1 = path_x->at(last) - path_x->at(last - 1);
			int delta_x2 = path_x->at(last - 1) - path_x->at(last - 2);
			int delta_y1 = path_y->at(last) - path_y->at(last - 1);
			int delta_y2 = path_y->at(last - 1) - path_y->at(last - 2);
	    
			if (delta_x1 == delta_x2 && delta_y1 == delta_y2) {
				bonus += 2; //2;
			}
		}
		// Reduce bonus for first and last rows
		if(pos_y == 0 || pos_y == 4)
			bonus += -4;
		value += maze[pos_y][pos_x];
		depth++;
    
		// If we reach the maximum depth, see if the path is better than the previous best one
		if (depth > max_depth) {
			
			// Update the best path if the new one is better
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
		// If we didn't reach max depth, keep searching
		else {
			// Search right
			path_x->push_back(pos_x + 1);
			path_y->push_back(pos_y);
			search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
			path_x->pop_back();
			//path_y->pop_back();
	    
			// Search left
			path_x->push_back(pos_x - 1);
			//path_y->push_back(pos_y);
			search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
			path_x->pop_back();
			path_y->pop_back();
	    
			// Search down
			path_x->push_back(pos_x);
			path_y->push_back(pos_y + 1);
			search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
			//path_x->pop_back();
			path_y->pop_back();
	    
			// Search up
			//path_x->push_back(pos_x);
			path_y->push_back(pos_y - 1);
			search_paths(r1, path_x, path_y, value, bonus, depth, max_depth, path_x->back(), path_y->back(), r2x, r2y);
			path_x->pop_back();
			path_y->pop_back();
		}
	}
}

/*
* Tries to make a move for r1, if it can't, generate more moves
* */
void move(path *r1, path *r2) {
	int ret = -1;
	
	// Try to reserve a move
	if (r1->moves_x->size() > 0) { //&& !(r1->moves_x->front() == r2->curr_x))
		ret = robot->reserveMap(r1->moves_x->front(), r1->moves_y->front());
		printf("Ret: %d: %d %d \n", ret, r1->moves_x->front(), r1->moves_y->front());
	}
	
	if (0 == ret) {
		// Consume the move we are making from the queue
		r1->curr_x = r1->moves_x->front();
		r1->curr_y = r1->moves_y->front();
		r1->moves_x->erase(r1->moves_x->begin());
		r1->moves_y->erase(r1->moves_y->begin());
		
		// Move to cell if we are not debugging
		robotPose->moveToCell(r1->curr_x, r1->curr_y);
	
		// Sleep if we are debugging
		//sleep(1);
		
		robot->updateMap(r1->curr_x, r1->curr_y);
		
		//if (maze[r1->curr_y][r1->curr_x] > 0) {
			//r1_score += maze[r1->curr_y][r1->curr_x];
			//maze[r1->curr_y][r1->curr_x] = 0;
		//}
	}
	// Give up and try generating new moves
	else {
		get_moves(r1, r2);
		move(r1, r2);
	}
}

/*
* Retrieve map data from server and update the map
* */
void getMap() {
	assert(NULL!= robot);
	map_obj_t * mapList = robot->getMap(&score1, &score2);
	
	if (mapList == NULL) {
		printf("Error getting map\n");
	}
	
	int robot1 = 0;
	int robot2 = 0;
	if (robotPose->player == 1) {
		robot2 = -1;
	}
	else if (robotPose->player == 2) {
		robot1 = -1;
	}
	
	// Read the map into the matrix from the linked list
	// Keep a count of the number of moves that are left
	moves_left = 0;
	for (int y = 0; y < 5; y++) {
		for (int x = 0; x < 7; x++) {
			if (MAP_OBJ_POST == (int)mapList->type) {
				maze[y][x] = -1;
			} 
			else if (MAP_OBJ_ROBOT_1  == (int)mapList->type) {
				maze[y][x] = robot1;
			}
			else if (MAP_OBJ_ROBOT_2 == (int)mapList->type) {
				maze[y][x] = robot2;
			}
			else if (MAP_OBJ_PELLET == (int)mapList->type) {
				maze[y][x] = mapList->points;
				moves_left++;
			}
			else if (MAP_OBJ_EMPTY == (int)mapList->type) {
				maze[y][x] = 0;
			}
			else if (MAP_OBJ_RESERVE_1 == (int)mapList->type) {
				maze[y][x] = robot1;	
			}
			else if (MAP_OBJ_RESERVE_2 == (int)mapList->type) {
				maze[y][x] = robot2;	
			}
			else {
				maze[y][x] = -999; 
			}
			
			// North star is bad in this row so we don't go there
			//if (y==0) {
			//	maze[y][x] = -9;
			//}
			
			mapList = mapList->next;
		}
	}
}