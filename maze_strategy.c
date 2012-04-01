#define R1 0
#define R2 1

void r1_get_moves();
void r2_get_moves();
void r1_move();
void r2_move();
void move(int robot);
int valid_move(int rx, int ry, int mx, int my, int r2x, int r2y);

int moves_left = 27;
int r1_pos[2];
int r2_pos[2];
int r1_score = 0;
int r2_score = 0;

int r1_moves_x[35];
int r1_moves_y[35];
int r1_move = 0;
int r1_num_moves = 0;

int r2_moves_x[35];
int r2_moves_y[35];
int r2_move = 0;
int r2_num_moves = 0;


int main() {
    
    r1_pos[0] = 0;
    r1_pos[1] = 2;
    
    r2_pos[0] = 6;
    r2_pos[1] = 2;
    
    int maze[5][7];

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
	r1_move();
	r2_move();
    }
}

void r1_get_moves() {
    r1_move = 0;
    r1_num_moves = 0;
    // come up with a sequence of moves and put them into r1_moves_x and r1_moves_y
}

void r2_get_moves() {
    r2_move = 0;
    r2_num_moves = 0;
    // come up with a sequence of moves and put them into r2_moves_x and r2_moves_y
}

void r1_move() {
    if (r1_move >= r1_num_moves) { // if there are no moves left, generate more
	r1_get_moves();
    }
    else {
	move(R1); // try to make a move
    }
    moves_left--;
}

void r2_move() {
    if (r2_move >= r2_num_moves) {
	r2_get_moves();
    }
    else {
	move(R2);
    }
    moves_left--;
}

void move(int robot) {
    if (robot == R1) { // if the move is valid, make it
	if (valid_move(r1_pos[0], r1_pos[1], r1_moves_x[r1_move], r1_moves_y[r1_move], r2_pos[0], r2_pos[1])) {
	    r1_pos[0] = r1_moves_x[r1_move];
	    r1_pos[1] = r1_moves_y[r1_move];
	    r1_move++;
	}
	else { // give up and try generating new moves
	    r1_move = r1_num_moves;
	    r1_move();
	}
    }
    else if (robot == R2) {
	if (valid_move(r2_pos[0], r2_pos[1], r2_moves_x[r2_move], r2_moves_y[r2_move], r1_pos[0], r1_pos[1])) {
	    r2_pos[0] = r2_moves_x[r2_move];
	    r2_pos[1] = r2_moves_y[r2_move];
	    r2_move++;
	}
	else {
	    r2_move = r2_num_moves;
	    r2_move();
	}
    }
}

int valid_move(int rx, int ry, int mx, int my, int r2x, int r2y) {
    return abs(rx - mx) + abs(ry - my) == 1 && !(mx == r2x && my == r2y);
}