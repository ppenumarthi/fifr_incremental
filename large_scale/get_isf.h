//Header file defining the functions needed to calculate ISF entries at each node.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#define MxSz 320 //Static size of the arrays used in the program, which represents the maximum number of nodes in the graph.
#define infinity 65535 //Some very high valued constant used to determine minimum costs between two nodes.
//#define MaxCombinations 2^MxSz

// 2 level debugging, lists out everything that we need
#ifndef DEBUG_2
#define DEBUG_2 0
#endif

// 1 level debugging, lists only the important events
#ifndef DEBUG_1
#define DEBUG_1 1
#endif

// KeyLinks level debugging, lists only the important events
#ifndef DEBUG_KL
#define DEBUG_KL 1
#endif



void print_ISF(const short ISF[MxSz][MxSz][MxSz], const short cost[MxSz][MxSz], const int n, const int state);

/* Equivalent to memset for multiple dimensional arrays */
void reset_5Darray(short KL[MxSz][MxSz][MxSz][MxSz][MxSz], int reset);

void reset_3Darray(short KL[MxSz][MxSz][MxSz], int reset);

void reset_2Darray(short NH[MxSz][MxSz], int reset);

void reset_1Darray(short array[MxSz], int reset);
void reset_1Darray(int array[MxSz], int reset);
/* Calculate minimum costs and next hops according to Dijkstra's algorithm */
void dij(int n,int v, const short cost[MxSz][MxSz], int dist[], short NextHop[MxSz][MxSz]);

/*
    Takes an ISF table and using the KL matrix, removes them from the cost matrix to compute the ISF table for all d on j->i
*/
void makeISF(short KL[MxSz][MxSz][MxSz], short cost[MxSz][MxSz], int dist[MxSz], short SubTree[MxSz], int n, int i, int j, short ISF[MxSz][MxSz][MxSz]);

void makeSPT(int source, int dest, short NextHop[MxSz][MxSz], short SPT[MxSz][MxSz]);

void makeSubTree_v2(int subvertex, short SPT[MxSz][MxSz], int n, short SubTree[MxSz]);

// ORIGNAL ISF from PAPER
void runISF_orig(short cost[MxSz][MxSz], int n, short ISF[MxSz][MxSz][MxSz]);

//Print the edge costs taken from the input file (In other words: used to verify if the input is same as what is given) 
void print_cost(short cost[MxSz][MxSz], int n);

void get_isf(short cost[MxSz][MxSz], int n, short ISF[MxSz][MxSz][MxSz]);

void print_IIF(const short IIF[MxSz][MxSz], const int n, int state);

int getdist(int no_of_nodes, int src, int dest, short cost[MxSz][MxSz]);

void reset_keylinks(int no_of_nodes);
