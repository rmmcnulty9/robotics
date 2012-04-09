#include "shared_constants.h"
#include <stdlib.h>
#define R1 0
#define R2 1

void get_moves(path *r1, path *r2);
void try_move(path *r1, path *r2);
void move(path *r1, path *r2);
int valid_move(path *r1, path *r2);

int moves_left = 27;
int r1_score = 0;
int r2_score = 0;

path **paths;
path *r1;
path *r2;

int maze[5][7];


int main() {
	paths = new path*[2];
	paths[0] = (path*)malloc(sizeof(path));
	paths[0]->moves_x = new list<int>();
	paths[0]->moves_y = new list<int>();
	
	paths[1] = (path*)malloc(sizeof(path));
	paths[1]->moves_x = new list<int>();
	paths[1]->moves_y = new list<int>();
    
	paths[0]->curr_x = 0;
	paths[0]->curr_y = 2;
    
	paths[1]->curr_x = 6;
	paths[1]->curr_y = 2;

	maze[0][0] = 5;
	maze[0][1] = 3;
	maze[0][2] = 4;
	maze[0][3] = 1;
	maze[0][4] = 6;
	maze[0][5] = 2;
	maze[0][6] = 10;
    
	maze[1][0] = 1;
	maze[1][1] = -1;
	maze[1][2] = 6;
	maze[1][3] = -1;
	maze[1][4] = 1;
	maze[1][5] = -1;
	maze[1][6] = 3;
    
	maze[2][0] = 0;
	maze[2][1] = 2;
	maze[2][2] = 5;
	maze[2][3] = 10;
	maze[2][4] = 5;
	maze[2][5] = 2;
	maze[2][6] = 0;
    
	maze[3][0] = 3;
	maze[3][1] = -1;
	maze[3][2] = 4;
	maze[3][3] = -1;
	maze[3][4] = 6;
	maze[3][5] = -1;
	maze[3][6] = 1;
    
	maze[4][0] = 10;
	maze[4][1] = 2;
	maze[4][2] = 6;
	maze[4][3] = 1;
	maze[4][4] = 4;
	maze[4][5] = 3;
	maze[4][6] = 5;
    
	while (moves_left > 0) {
		move(paths[0], paths[1]);
		move(paths[1], paths[0]);
	}
}

void get_moves(path *r1, path *r2) {
	r1->moves_x->clear();
	r1->moves_y->clear();
	// come up with a sequence of moves and put them into r2_moves_x and r2_moves_y
	
	

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
	if (valid_move(r1, r2)) {
		r1->curr_x = r1->moves_x->front();
		r1->curr_y = r1->moves_y->front();
		
		r1->moves_x->pop_front();
		r1->moves_y->pop_front();
    
		if (maze[r1->curr_y][r1->curr_x] > 0) {
			moves_left--;
			r1_score += maze[r1->curr_y][r1->curr_x];
			maze[r1->curr_y][r1->curr_x] = 0;
		}
	}
	else { // give up and try generating new moves
		r1->moves_x->clear();
		r1->moves_y->clear();
		try_move(r1, r2);
	}
}

int valid_move(path *r1, path *r2) {
	return !(r1->moves_x->front() == r2->curr_x && r1->moves_y->front() == r2->curr_y);
}