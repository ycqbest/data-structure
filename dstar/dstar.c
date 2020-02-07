/*
  Version 2-11-00
  Written by Bruce A. Maxwell

  This is an implementation of the D-star path planning algorithm based
  on the Focused D-Star paper by Anthony Stentz (CMU Robotics Institute)

  Revised 3-29-01 by Yi Guo
*/

#include <stdio.h>
#include <stdlib.h>
#include "dstar.h"

// This prints a list of the nodes to the screen
void            printNodeList(Node * list, char *name, void (*printfunc) (Node *));
void            printNodeList(Node * list, char *name, void (*printfunc) (Node *))
{
  Node           *p;

  printf("Printing list %s\n", name);
  p = list;
  while (p != NULL) {
    printfunc(p);
    p = p->next;
  }
}

Node           *insertOPEN(Node * openList, Node * newnode, double newG, double (*hcalc) (Node *), void (*printNode)(Node *));
Node           *insertOPEN(Node * openList, Node * newnode, double newG, double (*hcalc) (Node *), void (*printNode)(Node *))
{
  Node           *p, *q;


  if (newnode->state == NEW) {		       // set the k value for this node

    newnode->k = newG;
  }
  else if (newnode->state == OPEN) {	       // node is on OPEN already

    // update the k value if the new g value is lower
    newnode->k = newnode->k < newG ? newnode->k : newG;

    // node is on the open list, so delete it and re-insert it below
    p = (Node *)newnode->prev;
    q = (Node *)newnode->next;
    if (p != NULL)
      p->next = newnode->next;
    if (q != NULL)
      q->prev = newnode->prev;

    // check for the case of the node being at the head of the list
    if (newnode == openList)
      openList = newnode->next;

    newnode->next = NULL;
    newnode->prev = NULL;
  }
  else {
    // update the k value if the new G value is lower
    newnode->k = newnode->g < newG ? newnode->g : newG;
  }

  // calculate OPEN sort key
  newnode->g = newG;
  newnode->h = hcalc(newnode);
  newnode->f = newnode->k + newnode->h;
  newnode->state = OPEN;

  // now insert the state into the openList 

  // Test the case where openList is NULL
  if (openList == NULL) {
    newnode->next = openList;
    newnode->prev = NULL;
    return (newnode);
  }

  // Test the case where the new node is at the head of the list
  if ((newnode->f < openList->f) || ((newnode->f == openList->f) && (newnode->k < openList->k))) {
    newnode->next = openList;
    if(openList != NULL)
      openList->prev = newnode;
    newnode->prev = NULL;
    return (newnode);
  }

  // start the loop through the OPEN list
  p = openList;
  q = p->next;
  while (p != NULL) {

    if (q == NULL) {			       // end of the list, insert after p

      p->next = newnode;
      newnode->next = NULL;
      newnode->prev = p;
      return (openList);
    }

    if (newnode->f < q->f || ((newnode->f == q->f) && (newnode->k < q->k))) {
      // insert the node before p and after q
      newnode->next = q;
      if (q != NULL)
	q->prev = newnode;

      p->next = newnode;
      newnode->prev = p;

      return (openList);
    }

    p = (Node *) p->next;
    q = p->next;
  }

  return (openList);
}

#define LESS(a1, a2, b1, b2) ((a1) < (b1) ? 1 : ((a1) == (b1)) && ((a2) < (b2)) ? 1 : 0)
#define LESSEQ(a1, a2, b1, b2) ((a1) < (b1) ? 1 : ((a1) == (b1)) && ((a2) <= (b2)) ? 1 : 0)

/* 
 * 
 */
Node           *DStarSearch(Node ** initial, 
			    int numInitial, 
			    double (*gcalc) (Node *), 
			    double (*hcalc) (Node *),
			    int (*robotNode) (Node *), 
			    int (*neighbors) (Node *, Node **),
			    double (*cost) (Node *, Node *), 
			    double costR[2],  // (f = h + g, g) for the robot node, large values if never visited
			    void (*printNode) (Node *))
     //		    void (*drawArrow) (Node *, Node *))
{

  Node           *openList;
  Node           *closedList;
  Node           *current;
  Node           *p;
  Node           *path;
  Node           *neighbor[MAXNEIGHBORS];
  double          kold;
  double          fold;
  int             numNeighbors;
  long            i;
  static int      gblExpand = 0;
  static Node     *oldOpen = NULL;

  printf("Beginning search\n");

  // generate the open list
  openList = NULL;

  if(oldOpen != NULL) { // this is a recall of Dstar with new information

printNodeList(oldOpen, "oldOpen", printNode);

    // add the old open list nodes to the new open list so their h values are updated
    p = oldOpen;
    while(p != NULL) {
      p->state = CLOSED;
      oldOpen = p->next;

      openList = insertOPEN(openList, p, p->k, hcalc, printNode);
      
      p = oldOpen;
    }
  }
  // oldOpen is NULL at this point

  // generate the closed list
  closedList = NULL;

  // put the initial nodes on the open list
  for (i = 0; i < numInitial; i++)
    openList = insertOPEN(openList, initial[i], initial[i]->g, hcalc, printNode);

  //printNodeList(openList, "OPEN", printNode);

  while (openList != NULL) {

    // assume the open list is always sorted (robot doesn't move while D* is running)
    current = openList;
    openList = (Node *) openList->next;
    if (openList != NULL)
      openList->prev = NULL;
    gblExpand++;

    // kold = Get-KMIN()
    kold = current->k;
    fold = current->f;
    current->state = CLOSED;
    current->next = NULL;		       // need to reset this back to NULL

    current->prev = NULL;

    //printf("Current node: ");
    //printNode(current);

    /*  if(current->parent != NULL) {
      printf("Current parent: ");
      printNode(current->parent);
      }*/

    /*
    if(((Node*)current->parent)->parent != NULL) {
      printf("Current parent's parent: ");
      printNode(((Node *)current->parent)->parent);
    }
    */

    if(robotNode(current)) {
      costR[0] = current->h + current->g;
      costR[1] = current->g;
    } 

    // is the current node the goal node?
    if (robotNode(current) && current->k == current->g) { // robot node, and a LOWER node
      // If so, return a pointer to the parent node
      path = (Node *) current->parent;

      printf("Robot state reached with %d nodes expanded\n", gblExpand);

      // set oldOpen to keep around these nodes
      oldOpen = openList;

      // now return the path
      return (path);
    }

    // has the search gone past where it needs to go?
    if(!LESSEQ(fold, kold, costR[0], costR[1])) { // exit
      printf("Search terminated\n");
      oldOpen = openList;

      return(NULL);
    }

    numNeighbors = neighbors(current, neighbor);

    // if kold < g(X) then
    if (kold < current->g) {		       // check if any of the neighbors have a better path to the
					       // goal

      for (i = 0; i < numNeighbors; i++) {
	if(neighbor[i]->state == CLOSED && hcalc(neighbor[i]) != neighbor[i]->h)
	  continue;
	  
	if ((neighbor[i]->state != NEW) && LESSEQ(neighbor[i]->f, neighbor[i]->g, fold, kold) &&
	    (current->g > neighbor[i]->g + cost(neighbor[i], current))) {

	  // reset the back pointer to the better neighbor
	  current->parent = neighbor[i];

	  // calculate the new g value for the current node
	  current->g = neighbor[i]->g + cost(neighbor[i], current);
	}
      }
    }

    //printf("Node after neighbors: ");
    //printNode(current);

    /*if(current->parent != NULL) {
      printf("Parent after neighbors: ");
      printNode(current->parent);
      }*/

    if (kold == current->g) {		       // LOWER state

      //printf("Lower state\n");

      for (i = 0; i < numNeighbors; i++) {
	if ((neighbor[i]->state == NEW) ||
	((neighbor[i]->parent == current) && (neighbor[i]->g != current->g + cost(current, neighbor[i]))) ||
	 ((neighbor[i]->parent != current) && (neighbor[i]->g > current->g + cost(current, neighbor[i])))) {

	  // printf("Updated child cost\n");

	  // set the back pointer
	  neighbor[i]->parent = current;

	  // insert the neighbor into OPEN with the new G value
	  openList = insertOPEN(openList, neighbor[i], current->g + cost(current, neighbor[i]), hcalc, printNode);
	}
      }
    }
    else {				       // RAISE state

      //printf("Raise state\n");

      for (i = 0; i < numNeighbors; i++) {

	if ((neighbor[i]->state == NEW) ||
	((neighbor[i]->parent == current) && (neighbor[i]->g != current->g + cost(current, neighbor[i])))) {

	  //printf("inserted a neighbor with a new cost value\n");

	  // set the back pointer
	  neighbor[i]->parent = current;

	  // insert the neighbor into OPEN with the new g value
	  openList = insertOPEN(openList, neighbor[i], current->g + cost(current, neighbor[i]), hcalc, printNode);
	}
	else {
	  if ((neighbor[i]->parent != current) && (neighbor[i]->g > current->g + cost(current, neighbor[i]))) {

	    //printf("inserted self as a holding action\n");

	    // insert the current node into OPEN as a holding action until its neighbors are optimal
	    openList = insertOPEN(openList, current, current->g, hcalc, printNode);
	  }
	  else if ((neighbor[i]->parent != current) &&
		   (current->g > neighbor[i]->g + cost(neighbor[i], current)) &&
		   (neighbor[i]->state == CLOSED) && LESS(fold, kold, neighbor[i]->f, neighbor[i]->g)) {

	    //printf("inserted neighbor as a holding action\n");

	    // re-insert this CLOSED node since it is not optimal but already provides a better path
	    openList = insertOPEN(openList, neighbor[i], neighbor[i]->g, hcalc, printNode);
	  }
	}
      }
    }

    // draw the current node in the map here
    //drawArrow(current, current->parent);

    /*if(current->parent != NULL) {
      printf("New parent: ");
      printNode(current->parent);
      }*/

    // Test to see if we have expanded too many nodes without a solution
    if (gblExpand > MAXNODES) {
      printf("Expanded more than the maximum allowable nodes (%d). Terminating\n", gblExpand);

      return (NULL);
    }
  }					       // end of OPEN loop

  // if we got here, then there is no path to the goal
  oldOpen = openList;
  printNodeList(oldOpen, "oldOpen", printNode);

  return (NULL);
}
