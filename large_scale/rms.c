#include "get_isf.h"

int rms[MxSz] = {-1};
int optimized_rms[MxSz] = {-1};
int next_metric = -1;
int affected_dest[MxSz] = {0};
int affected_dest_count = 0;
short initialRSPT[MxSz][MxSz] = {0};
short targetRSPT[MxSz][MxSz] = {0};
short NextHop[MxSz][MxSz] = {0};
int rms_index = 0;
int optimized_rms_index = 0;

extern short orig_cost[MxSz][MxSz];
extern int no_of_nodes;


void optimizeRMS(int dest, int node1, int node2)
{
    //printf("Entered OptimizeRMS function\n");
    int temp_count_rms = 0;
    
    int initial_metric = rms[temp_count_rms++];
    int final_metric = rms[rms_index-1];
    int old_metric = initial_metric;
    int current_metric = rms[temp_count_rms++];
    
    short temp_cost[MxSz][MxSz];
    short temp_nexthop[MxSz][MxSz];
    short SPT[MxSz][MxSz];
    reset_2Darray(SPT, -1);
    reset_2Darray(temp_nexthop, -1);

    short temp_cost2[MxSz][MxSz];
    short temp_nexthop2[MxSz][MxSz];
    short SPT2[MxSz][MxSz];
    reset_2Darray(SPT2, -1);
    reset_2Darray(temp_nexthop2, -1);
    
    short merged_SPT[MxSz][MxSz];    
    reset_2Darray(merged_SPT, -1);
    
    int dist[MxSz];
    
    optimized_rms[optimized_rms_index++] = old_metric;

    while (current_metric != final_metric)
    {
        memcpy(temp_cost, orig_cost,  MxSz*MxSz*sizeof(int));        
        memcpy(temp_cost2, orig_cost,  MxSz*MxSz*sizeof(int));

        temp_cost[node1][node2] = old_metric;
        temp_cost[node2][node1] = old_metric;
        
        temp_cost2[node1][node2] = current_metric;
        temp_cost2[node2][node1] = current_metric;
        
        // Make SPT1
        reset_1Darray(dist, 0);
        dij(no_of_nodes, dest, temp_cost, dist, temp_nexthop);
        for( int v =0; v < no_of_nodes; v++)
        {
            if (v != dest)
            makeSPT(dest, v, temp_nexthop, SPT);        
        }
        
        // Make SPT2   
        reset_1Darray(dist, 0);
        dij(no_of_nodes, dest, temp_cost2, dist, temp_nexthop2);
        for( int v =0; v < no_of_nodes; v++)
        {
            if (v != dest)
            makeSPT(dest, v, temp_nexthop2, SPT2);        
        }

        // Merge SPT and SPT2
        for( int v =0; v < no_of_nodes; v++)
        {
            for( int u =0; u < no_of_nodes; u++)
            {
                if (SPT[u][v] == 1 || SPT2[u][v] == 1)
                    merged_SPT[u][v] = 1;
            }
        }
        
        //printf("Check for loops now\n");
        // verify if a loop is possible in this merged_SPT array
        int does_cycle = 0;
        for (int src = 0; src < no_of_nodes; src++)
        {
            for (int dest = 0; dest < no_of_nodes; dest++)
            {
                int visited_nodes[MxSz] = {0};
                int prev_next =src, next = -1;
                int i;
                
                //printf("here.....\n");
                while (next != dest && prev_next != next)
                {
                    prev_next = next;
                    //printf("here2...%c..%c\t", i+'A', next+'A');
                    if (next != -1)
                        i = next;
                    else
                        i = src;
                    // Doing Depth First Search now. 
                    // Cycle occurs if next node is repeated.
                    for(int j=0; j<no_of_nodes; j++)
                    {
                        if (merged_SPT[i][j] == 1)
                        {
                            next = j;
                            if(visited_nodes[j] == 1)
                                visited_nodes[j] = 1;
                            else
                                does_cycle = 1;
                        }
                    }
                }
            }
        }
        

        //printf("Check for loops is completed now\n");
        if(does_cycle != 0)
        {
            optimized_rms[optimized_rms_index++] = old_metric;
            old_metric = current_metric;
        }
        
        // Update the current and old_metrics for the next iterations
        current_metric = rms[temp_count_rms++];
    }
    
}

void swap(int *xp, int *yp)
{
    int temp = *xp;
    *xp = *yp;
    *yp = temp;
}
 
// Bubble sort in an aray
void bubbleSort(int no_of_elements, int array[])
{
    int i, j;
    for (i = 0; i < no_of_elements-1; i++)
    {
        for (j = 0; j < no_of_elements-i-1; j++) 
        {
            if (array[j] > array[j+1])
            {
                swap(&array[j], &array[j+1]);
            }
        }
    }       
}

int get_nextmetric()
{
    if (rms_index == 0)
        return -1;
    next_metric++;
    if( (next_metric > MxSz) || (optimized_rms[next_metric] == 0) )
        return -1;
    else
        optimized_rms[next_metric];        
}

int get_next_rms_metric()
{
    if (rms_index == 0)
        return -1;
    next_metric++;
    if( (next_metric > MxSz) || (rms[next_metric] == 0) )
        return -1;
    else
        rms[next_metric];        
}

int pathlength(int source, int dest, int state)
{
    if( state == 0)
    {
        // Use InitialRSPT
    }
    else
    {
        // Use TargetRSPT
    }
}

/* makeSPT function written by Aaron actually calculates the RSPT
 * RSPT and SPT will be same for  symmetric link topology
 */
void makeRSPT(int source, int dest, short NextHop[MxSz][MxSz], short SPT[MxSz][MxSz])
{
	if( source == dest )
    {    
		// Do nothing
	}
    else if (NextHop[source][dest] == infinity)
    {
		SPT[source][dest] = 0;
	}
    else if ( NextHop[source][dest] != dest)
    {
		makeSPT(NextHop[source][dest], dest, NextHop, SPT);
	}
    else
    {
		SPT[source][dest] = 1;
	}
}


/* SPT will be created to "dest"
 * This is a tree containing all the shortest paths from the node X
 * RSPT will be created to "source"
 * RSPT(X) is a tree containing all the shortest paths from the nodes of the network graph towards X
 * 
 * RSPT(link A-->B) will provide shortest path tree of all nodes in the network to A 
 * */
void computeRSPT(int dest, int node1, int node2, int cost_of_link, int updated)
{
	short temp_cost[MxSz][MxSz];
	memcpy(temp_cost, orig_cost,  MxSz*MxSz*sizeof(int));
	temp_cost[node1][node2] = cost_of_link;
	temp_cost[node2][node1] = cost_of_link;
	
	for (int dst = 0; dst < no_of_nodes; dst++)
	{
		if (dst == dest)
			continue;
			
		int temp_dist[MxSz];
		short temp_nextHop[MxSz][MxSz];
		
		dij(no_of_nodes, dst, temp_cost, temp_dist, temp_nextHop);

        if (updated == 0)
            makeRSPT(dst, dest, temp_nextHop, initialRSPT);
        else if (updated == 1)
            makeRSPT(dst, dest, temp_nextHop, targetRSPT);		
	}
}

void getRMS_old( int dest, int node1, int node2, int dead_weight)
{
    int rms_index = 0;
    rms[rms_index++] = orig_cost[node1][node2];
    rms[rms_index++] = orig_cost[node1][node2] + 1;
    rms[rms_index++] = dead_weight;
    
    computeRSPT (dest, node1, node2, orig_cost[node1][node2], 0);
    computeRSPT (dest, node1, node2, dead_weight, 1);
    
    for (int node =0; node < no_of_nodes; node++)
    {
        int pathlength_initial = pathlength(node, dest, 0);
        int pathlength_target = pathlength(node, dest, 1);
        
        if (pathlength_initial != pathlength_target)
        {
            int keymetric = orig_cost[node1][node2] + pathlength_initial - pathlength_target;
            if (keymetric != dead_weight)
            {
                rms[rms_index++] = keymetric;
                rms[rms_index++] = keymetric + 1;
            }
        }
    }
}

void getRMS (int dest, int node1, int node2, int dead_weight)
{
    //printf("orig_cost and dead_weight as %d , %d" ,orig_cost[node1][node2], dead_weight);
    rms[rms_index++] = orig_cost[node1][node2];
    rms[rms_index++] = dead_weight;

    short temp_cost[MxSz][MxSz];
    memcpy(temp_cost, orig_cost,  MxSz*MxSz*sizeof(int));
    temp_cost[node1][node2] = dead_weight;
    temp_cost[node2][node1] = dead_weight;
     
    for (int node =0; node < no_of_nodes; node++)
    {
        int pathlength_initial = getdist(no_of_nodes, node, dest, orig_cost);
        int pathlength_target = getdist(no_of_nodes, node, dest, temp_cost);
        
        if (pathlength_initial != pathlength_target)
        {
            // pathlength_initial - pathlength_target can be +ve  or -ve.
            int keymetric = (pathlength_initial - pathlength_target);
            if (keymetric < 0) 
                keymetric = keymetric * (-1);
            keymetric += orig_cost[node1][node2];
            if (keymetric != dead_weight)
            {
                int is_repeat = 0;
                for(int i=0; i<rms_index; i++)
                {
                    if (rms[i] == keymetric)
                    {
                        is_repeat = 1;
                        continue;
                    }
                }
                if (is_repeat == 1)
                    continue;

                //printf("\tadding: %d and %d", keymetric, keymetric+1);
                rms[rms_index++] = keymetric;
                rms[rms_index++] = keymetric + 1;
            }
        }
    }
    //printf("Done RMS\n");
}


/* Populate affected destinations for each link separately
 * I used IIF entries to caclculate follow(A->B, SPT_{init{}(A) ).
 * This may not be the correct implementation.
 */
void populate_affected_destinations(int node1_failed_link, int node2_failed_link)
{
    for (int dest =0 ; dest < no_of_nodes; dest++)
        if( NextHop[node1_failed_link][dest] == node2_failed_link )
        {
            affected_dest[dest] = 1;
            affected_dest_count++;
        }
}

/* Link is considered as node1-->node2
 * Dead weight will be calculated as thw eight after which this link becomes useless.
 * Function implementation follows the algortithm in
 * http://ieeexplore.ieee.org/document/4215601/
 */
void make_route_metric_sequence(int node1_failed_ink, int node2_failed_link, int dead_weight)
{
    reset_1Darray(affected_dest, 0);
    affected_dest_count = 0;
    /* Find the list of affected destinations
     * If link node1--> node2 is used for reaching destination 'd'; d will be affected.
     */
    populate_affected_destinations(node1_failed_ink, node2_failed_link);
    
    
    for (int dest = 0; dest < no_of_nodes; dest++)
    {
        if (affected_dest[dest] == 1)
        {
            // Reset previously generated RMS destinaitons.
            reset_1Darray(rms, -1);
            next_metric = -1;
            rms_index = 0;

            getRMS(dest, node1_failed_ink, node2_failed_link, dead_weight);
            //printf("sorting");
            bubbleSort(rms_index, rms);
            
            //printf("\n--------Optimizing RMS for dest: %c-----------", dest+'A');
            
            optimizeRMS(dest, node1_failed_ink, node2_failed_link);
            bubbleSort(optimized_rms_index, optimized_rms);            
        }        
    }
    
}

