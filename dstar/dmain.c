/*
	This file contains a simple example of using the D* routine
	
	The task is coded up to find a path on a regular grid between
	any two points.  The grid values can be integers from 0 to 100.
	A specified number of rectangular obstacles can be defined.
	
	Revised 3-29-01 by Yi Guo

*/

// Include files
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dstar.h"
#include <unistd.h>

// global variables: goal configuration, initial configuration, obstacles
int gblGoal[2] = {50, 15};
int gblRobot[2] = {10, 5};
char gblImage[GRIDY + 1][GRIDX + 1];
//double elev[GRIDY][GRIDX];
		
// define the number of obstacles here
int gblNumObstacles=2;
#define gblMaxObstacles 3

// define the obstacle rectangles here: (top, left, bottom, right), origin in lower left
int gblObstacle[gblMaxObstacles][4] = { 17, 15, 0, 17,
					12, 5, 10, 15,
					18, 30, 10, 35};

// NodeInfo structure
typedef struct {
	int	x;
	int 	y;
} NodeInfo;

Node *gblGrid;
NodeInfo *gblInfo;

//double 

// Test for whether a point is in an obstacle or not
int inObstacle(int x, int y);
int inObstacle(int x, int y) {
	int i;
	
	// uncomment this to remove the obstacles
	// return(0);
	
	for(i=0;i<gblNumObstacles;i++) {
		if(y <= gblObstacle[i][0] && y >= gblObstacle[i][2] && 
			x >= gblObstacle[i][1] && x <= gblObstacle[i][3])
			return(1);
	}
	
	return(0);
}

double cost(Node *to, Node *from);
double cost(Node *to, Node *from) {
	double dx, dy;
	
	dx = ((NodeInfo *)to->nodeInfo)->x - ((NodeInfo *)from->nodeInfo)->x;
	dy = ((NodeInfo *)to->nodeInfo)->y - ((NodeInfo *)from->nodeInfo)->y;

	if(inObstacle(((NodeInfo *)to->nodeInfo)->x, ((NodeInfo *)to->nodeInfo)->y))
		return(1e+7 + sqrt(dx*dx + dy*dy));
	else
		return(sqrt(dx*dx + dy*dy));
}

// define the g function as parent plus a step
double gfunction(Node *p);
double gfunction(Node *p) {
	Node *q;
	
	if(p == NULL)
		return(0.0);
		
	if(p->parent == NULL)
		return(0.0);

	// This uses movement from the initial state
	q = (Node *)p->parent;

	return(q->g + cost(q, p));
}

// define the h function as Euclidean distance to the robot
double hfunction(Node *p);
double hfunction(Node *p) {
	NodeInfo *ni;
	double h, dx, dy;
	
	if(p == NULL)
		return(1e+7);
	
	ni = (NodeInfo *)p->nodeInfo;
	
	// Uncomment this to get the basic D* with no focusing
	// return(0);
	
	// Uncomment this to use Euclidean distance
	dx = gblRobot[0] - ni->x;
	dy = gblRobot[1] - ni->y;
	h = sqrt(dx * dx + dy * dy);
	
	return(h);
}

	

// define the robotNode function
int robot(Node *p);
int robot(Node *p) {
	NodeInfo *ni;
	
	ni = (NodeInfo *)p->nodeInfo;
	
	if(ni->x == gblRobot[0] & ni->y == gblRobot[1])
		return(1);
	
	return(0);
}

// define the goalNode function
int goal(Node *p);
int goal(Node *p) {
	NodeInfo *ni;
	
	ni = (NodeInfo *)p->nodeInfo;
	
	if(ni->x == gblGoal[0] & ni->y == gblGoal[1])
		return(1);
	
	return(0);
}


// conventional direction coding for 8-connectedness
#define EAST 0
#define NORTHEAST 1
#define NORTH 2
#define NORTHWEST 3
#define WEST 4
#define SOUTHWEST 5
#define SOUTH 6
#define SOUTHEAST 7

// define the children function
int getNeighbors(Node *parent, Node **neighbor);
int getNeighbors(Node *parent, Node **neighbor) {
	NodeInfo *ni;
	int i, posx, posy;
	int deltax[8] = {1, 1, 0, -1, -1, -1, 0, 1};
	int deltay[8] = {0, 1, 1, 1, 0, -1, -1, -1};
	int numNeighbors;
	
	ni = (NodeInfo *)parent->nodeInfo;
			
	// build all of the legal children	
	numNeighbors = 0;
	for(i=0;i<8;i++) {
		// calculate the potential position of the next child
		posx = ni->x + deltax[i];
		posy = ni->y + deltay[i];
		
		// bounds check
		if(posx >= 0 && posx < GRIDX && posy >= 0 && posy < GRIDY) {
			// obstacle check
//			if(!inObstacle(posx, posy)) {
				// node has passed the tests: add it to the array of neighbors
				neighbor[numNeighbors++] = &(gblGrid[posy*GRIDX + posx]);
//			}
		}
	}
	
	return(numNeighbors);
}

// Free node function
void freeNode(Node *p);
void freeNode(Node *p) {
	NodeInfo *ni;
	
	ni = (NodeInfo *)p->nodeInfo;
	ni->x = ni->y = -1;
	p->parent = NULL;
	p->next = NULL;
	p->state = NEW;
}

// Test for node equality
int nodeEqual(Node *a, Node *b);
int nodeEqual(Node *a, Node *b) {
	
	if(a == NULL && b == NULL)
		return(1);
	else if(a == NULL)
		return(0);
	else if(b == NULL)
		return(0);
	
	if(((NodeInfo *)(a->nodeInfo))->x != ((NodeInfo *)(b->nodeInfo))->x)
		return(0);

	if(((NodeInfo *)(a->nodeInfo))->y != ((NodeInfo *)(b->nodeInfo))->y)
		return(0);
	
	// if we're here, the x and y values are the same
	return(1);
}

// simple function to print a node
void printNode(Node *p);
void printNode(Node *p) {
	printf("Node %05d: f %.2lf h %.2lf g %.2lf k %.2lf (%4d, %4d)\n", p->id, p->f, p->h, p->g, p->k, 
	       ((NodeInfo *)p->nodeInfo)->x, ((NodeInfo *)p->nodeInfo)->y);
}

void drawArrow(Node *child, Node *parent);
void drawArrow(Node *child, Node *parent) {
	NodeInfo *ci, *pi;
	int diffx, diffy;
	char c;
	long i;
	static int reset = 0;
	
	reset--;
	
	ci = (NodeInfo *)child->nodeInfo;
	if(parent == NULL)
		c = 'G';
	else {
		pi = (NodeInfo *)parent->nodeInfo;

		diffx = pi->x - ci->x;
		diffy = pi->y - ci->y;
		
		if(diffx == 0)
			c = '|';
		else if(diffy == 0)
			c = '-';
		else if(diffx * diffy >       0)
			c = '/';
		else
			c = '\\';
	}
	gblImage[ci->y][ci->x] = c;
	
	if(reset <= 0) {
		for(i=GRIDY-1;i>=0;i--)
			printf("%s\n", gblImage[i]);
		
		printf("ENTER # of iterations: ");
		scanf("%d", &reset);	
	}
}


// main function
main() {
	Node *root;
	NodeInfo *ni;
	Node *path, *p, *q;
	Node *initial[50];
	int numInitial;
	int step;
	int i, j, k, lo, hi, left, right;
	FILE *fp;
	double pathcost;
	double costR[2];

	
	// initialize the image
	for(i=0;i<GRIDY;i++) {
		for(j=0;j<GRIDX;j++) {
			gblImage[i][j] = '.';
		}
		gblImage[i][j] = '\0';
	}
	//gblImage[gblRobot[1]][gblRobot[0]] = 'R';

	// Put in the obstacle(s)
	for(k=0;k<gblNumObstacles;k++) {
		for(i=gblObstacle[k][2];i<=gblObstacle[k][0];i++) {
			for(j=gblObstacle[k][1];j<=gblObstacle[k][3];j++)
				gblImage[i][j] = '#';
		}
	}
	

       	gblImage[gblGoal[1]][gblGoal[0]] = 'G';
	gblImage[gblRobot[1]][gblRobot[0]] = 'R';
	
	// draw the initial image
	/*    	for(i=GRIDY-1;i>=0;i--) {
	  printf("%s\n", gblImage[i]);
	  }*/
	
       	// allocate the grid of nodes
	gblGrid = (Node *)malloc(sizeof(Node) * GRIDX * GRIDY);
	gblInfo = (NodeInfo *)malloc(sizeof(NodeInfo) * GRIDX * GRIDY);
	
	// initialize each grid cell
	for(i=0;i<GRIDY * GRIDX;i++) {
		gblGrid[i].nodeInfo = &(gblInfo[i]);
		gblGrid[i].next = NULL;
		gblGrid[i].prev = NULL;
		gblGrid[i].parent = NULL;
		gblGrid[i].state = NEW;
		gblGrid[i].id = i * GRIDX;
		gblInfo[i].x = i % GRIDX;
		gblInfo[i].y = i / GRIDX;
	}
	
	// setup the root node
	root = &(gblGrid[gblGoal[1]*GRIDX + gblGoal[0]]);
		
	root->g = 0;
	root->h = hfunction(root);
	root->f = root->g + root->h;

	// put it in the initial array
	initial[0] = root;
	numInitial = 1;

	costR[0] = costR[1] = 1e+7;

	// call the D* algorithm
	path = DStarSearch(initial, numInitial, gfunction, hfunction, robot, getNeighbors, cost, costR, printNode);
	
	// D* returned failure (couldn't reach the robot's location)
	if(path == NULL) {
		printf("No path found, terminating\n");
		return(0);
	}
	
	// otherwise, we had a successful search
	p = path;
	step = 0;
	while(p != NULL) {
		ni = (NodeInfo *)p->nodeInfo;
		printf("Step %03d: (%4d, %4d)\n", step++, ni->x, ni->y);
		gblImage[ni->y][ni->x] = 'x';
		p = (Node *)p->parent;
	}
	gblImage[gblGoal[1]][gblGoal[0]] = 'G';
	gblImage[gblRobot[1]][gblRobot[0]] = 'R';
	
       	// Put in the obstacle(s)
	for(k=0;k<gblNumObstacles;k++) {
		for(i=gblObstacle[k][2];i<=gblObstacle[k][0];i++) {
			for(j=gblObstacle[k][1];j<=gblObstacle[k][3];j++)
				gblImage[i][j] = '#';
		}
	}
	
	for(i=GRIDY-1;i>=0;i--) {
		printf("%s\n", gblImage[i]);
	}
	
	printf("Finished with initial search\n");
	sleep(4);
	
	// increment the number of obstacles: first increment
	gblNumObstacles++;

	// robot change positions
	gblRobot[0]=12;
	gblRobot[1]=15;

	// redraw the grid for the second search
	for(i=0;i<GRIDY;i++) {
		for(j=0;j<GRIDX;j++) {
		  gblImage[i][j] = gblImage[i][j] == 'x' ? 'o' : gblImage[i][j];
		}
	}


	// redraw the obstacles
	for(k=0;k<gblNumObstacles;k++) {
		for(i=gblObstacle[k][2];i<=gblObstacle[k][0];i++) {
			for(j=gblObstacle[k][1];j<=gblObstacle[k][3];j++)
				gblImage[i][j] = '#';
		}
		}

	// redraw the robot and goal states
	gblImage[gblGoal[1]][gblGoal[0]] = 'G';
	gblImage[gblRobot[1]][gblRobot[0]] = 'R';

	for(i=GRIDY-1;i>=0;i--) {
		printf("%s\n", gblImage[i]);
	}

	printf("Replace initial search path with old mark\n");
	sleep(4);

	// get the new obstacle ready to go
	lo = gblObstacle[gblNumObstacles-1][2];
	hi = gblObstacle[gblNumObstacles-1][0];
	left = gblObstacle[gblNumObstacles-1][1];
	right = gblObstacle[gblNumObstacles-1][3];
	for(k=0,i=lo;i<=hi;i++) {
	  for(j=left;j<=right;j++) {
	    if(i == lo || i == hi || j == left || j == right) {
	      if(gblGrid[i*GRIDX + j].parent != NULL) {
		initial[k] = &(gblGrid[(i)*GRIDX + j]);
		// leave the old f and g values and the next/prev values
		k++;
	      }
	    }
	  }
	}
	numInitial = k;

	costR[0] = gblGrid[gblRobot[1]*GRIDX + gblRobot[0]].f;
	costR[1] = gblGrid[gblRobot[1]*GRIDX + gblRobot[0]].g;

	// call the D* algorithm
	path = DStarSearch(initial, numInitial, gfunction, hfunction, robot, getNeighbors, cost, costR, printNode);
	
	// D* returned failure (couldn't reach the robot's location)
	if(path == NULL) {

	  // follow the path from the robot node
	  p = &(gblGrid[gblRobot[1] * GRIDX + gblRobot[0]]);

	  if(p->parent != NULL) {

	    // try following this path
	    pathcost = 0.0;
	    while(p != NULL) {
	      if(p->parent != NULL)
		pathcost += cost(p->parent, p);
	      p = (Node *)p->parent;
	    }
	    printf("pathcost = %.2lf\n", pathcost);

	    if(pathcost < 1e+7) {
	      // we had a successful search
	      p = &(gblGrid[gblRobot[1] * GRIDX + gblRobot[0]]);
	      step = 0;
	      while(p != NULL) {
		ni = (NodeInfo *)p->nodeInfo;
		gblImage[ni->y][ni->x] = 'x';
		p = (Node *)p->parent;
	      }
	    }
	    else {
	      printf("No free path exists\n");
	    }
	  }
	  else {
	    printf("No path found\n");
	  }
	}
	else {
	  p = path;
	  pathcost = 0.0;
	  while(p != NULL) {
	    if(p->parent != NULL)
	      pathcost += cost(p->parent, p);
	    p = (Node *)p->parent;
	  }
	  printf("pathcost = %.2lf\n", pathcost);

	  if(pathcost < 1e+7) {
	    // we had a successful search
	    p = path;
	    step = 0;
	    while(p != NULL) {
	      ni = (NodeInfo *)p->nodeInfo;
	      gblImage[ni->y][ni->x] = 'x';
	      p = (Node *)p->parent;
	    }
	  }
	  else {
	    printf("No free path exists\n");
	  }
	}

	gblImage[gblGoal[1]][gblGoal[0]] = 'G';
	gblImage[gblRobot[1]][gblRobot[0]] = 'R';
	
	// Put in the obstacle(s)
	for(k=0;k<gblNumObstacles;k++) {
		for(i=gblObstacle[k][2];i<=gblObstacle[k][0];i++) {
			for(j=gblObstacle[k][1];j<=gblObstacle[k][3];j++)
				gblImage[i][j] = '#';
		}
	}
	
	
	for(i=GRIDY-1;i>=0;i--) {
		printf("%s\n", gblImage[i]);
	}

	printf("Path after one increment of obstacles\n");
	sleep(4);


	printf("Terminating\n"); 
	
	
	// delete the nodes
	free(gblGrid);
	free(gblInfo);
	
	
	return(0);
}
