// Include file for D-star search algorithm

// These are global parameters that constrain the search

#define MAXNODES  30000
#define MAXNEIGHBORS	25
#define GRIDX	60
#define GRIDY	20

// These are ueful enumerations
#define OPEN 1
#define NEW 0
#define CLOSED 2

typedef struct {
  long  id;
  int  state;   		// {OPEN, NEW, CLOSED}
  double g;
  double h;
  double f;
  double k;
  void *parent;			// D* backpointer
  void *next;			// used for linked list connections
  void *prev;			// used for linked list connections
  void *nodeInfo;
} Node;


// function prototypes
Node *DStarSearch(Node **initialList, int numInitial,
				  double (*gcalc)(Node *), 
				  double (*hcalc)(Node *),
				  int (*robotNode)(Node *), 
				  int (*neighbors)(Node *, Node **),
				  double (*cost)(Node *, Node *), 
		                  double costR[2],
				  void (*printNode)(Node *));
//	  void (*drawArrow)(Node *, Node *));
