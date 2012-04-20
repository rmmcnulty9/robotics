/*
 * maze_strategy Header
 *  Used to define functions and variables
 *  for the pathfinding algorithm
 *   Written by Greg Seaman, Adam Park, and Ryan McNulty
 */

#define R1 0
#define R2 1

// Function declarations
void getMap();
void get_moves(path *r1, path *r2);
void search_paths(path *r1, vector<int> *path_x, vector<int> *path_y, int value, int bonus, int depth, int max_depth, int pos_x, int pos_y, int r2x, int r2y);
void move(path *r1, path *r2);

int moves_left = 27;
int r1_score = 0;
int r2_score = 0;

// The true score of the game from the server
int score1 = 0, score2 = 0;

int path_found=0;

// Path data for each robot
path **paths;
path *r1;
path *r2;

int maze[5][7];
int maze_visited[5][7];
RobotPose *robotPose;
RobotInterface *robot;
map_obj_t *robot_map;