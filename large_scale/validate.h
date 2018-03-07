#include "get_isf.h"

short ISF[MxSz][MxSz][MxSz], ISF_new[MxSz][MxSz][MxSz];
short orig_cost[MxSz][MxSz], old_cost[MxSz][MxSz], new_cost[MxSz][MxSz];
int no_of_nodes;
short new_nextHop[MxSz][MxSz], old_nextHop[MxSz][MxSz];
short max_edge_cost = 0;

char buffer[320];

int check_loop(char nodes_state[320], int source_node, int dest_node, int node1_with_failed_link, int node2_with_failed_link);
int check_loop(int source_node, int dest_node, int node1_with_failed_link, int node2_with_failed_link);

void validate_loops(int node1_for_failed_link, int node2_for_failed_link);
void validate_loops_no_print(int node1_for_failed_link, int node2_for_failed_link);

void fifr_increasing_weights();
void fifr_increasing_weights_rms() ;

int get_dead_weight(int i, int j);
