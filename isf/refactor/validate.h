#include "get_isf.h"

int ISF[MxSz][MxSz][MxSz], ISF_new[MxSz][MxSz][MxSz];
int orig_cost[MxSz][MxSz], old_cost[MxSz][MxSz], new_cost[MxSz][MxSz];
int no_of_nodes;
int new_nextHop[MxSz][MxSz], old_nextHop[MxSz][MxSz];
int max_edge_cost = 0;

char buffer[64];

int check_loop(char nodes_state[64], int source_node, int dest_node, int node1_with_failed_link, int node2_with_failed_link);

void validate_loops(int node1_for_failed_link, int node2_for_failed_link);

void fifr_increasing_weights();
