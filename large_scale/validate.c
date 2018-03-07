#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "validate.h"
#include "rms.c"

int edge_node1;
int edge_node2;

//int check_loop(char nodes_state[64], int source_node, int dest_node, int node1_with_failed_link, int node2_with_failed_link)
int check_loop(int source_node, int dest_node, int node1_with_failed_link, int node2_with_failed_link)
{
    if(DEBUG_2) printf("check_loop(no_of_nodes: %d, src: %d, dest: %d, failed link is between %d and %d)\n",  no_of_nodes, source_node, dest_node, node1_with_failed_link, node2_with_failed_link);
	if(DEBUG_2) 
    {
	  printf("NODES_STATE is\n");
	  for(int k=0; k<no_of_nodes; k++)
	 	printf("%c", buffer[k]);
	  printf("\n");
	}
    
    if (source_node == dest_node || source_node ==-1 || dest_node == -1)
		return 0;
	
	int next_node = -1;
	int prev_node = -1;
	int curr_node = -1;

	int edge_visited_count[MxSz][MxSz] = {0};

	while(next_node != dest_node) 
	{
		// Current node is NULL: Only happens for the first time.
		if( curr_node == -1) 
		{
			curr_node = source_node;
		}
		else
		{
            if(DEBUG_2) printf("1111");
		    prev_node = curr_node;
			curr_node = next_node;
		}
		// Packet originated here (Same as IIF)
		if (prev_node == -1)
		{
			if (curr_node == node1_with_failed_link || curr_node == node2_with_failed_link)
			{
                if(DEBUG_2) printf("2222");
                next_node = new_nextHop[source_node][dest_node];
			}
            else
			{
				if(DEBUG_2) printf("3333");
                next_node = old_nextHop[source_node][dest_node];
			}
		}
		else
		{
			// State 1 means Updated i.e., ISF_new
			if (curr_node == node1_with_failed_link || curr_node == node2_with_failed_link)
			{
                if(DEBUG_2) printf("5555");
            	next_node = new_nextHop[curr_node][dest_node];
			}
			else if(buffer[curr_node] == 49) //ASCII for '1'
			{
				if(DEBUG_2) printf("6666");
                next_node = ISF_new[curr_node][prev_node][dest_node];
				// Aaron has this weird convention of curr_node, prev_node and dest_node in the list of array elements.
				// While it has to be ISF_new[prev_node][curr_node][dest_node];
			}
			// State 0 means NOT Updated i.e., ISF
			else if(buffer[curr_node] == 48) //ASCII for '0'
			{
				if(DEBUG_2) printf("7777");
                next_node = ISF[curr_node][prev_node][dest_node];
				// Aaron has this weird convention of curr_node, prev_node and dest_node in the list of array elements.
			}
		}
		
		
		if(curr_node == next_node)
		{
            if(DEBUG_2) printf("No path from %d to %d in NEW topology, node-state is %d\n", curr_node, dest_node, buffer[curr_node]);
			return 0;
		}
		/* Should check for loop here:: Not sure as of now
		 */
		if ( edge_visited_count[curr_node][next_node] == 0)
		{
			if(DEBUG_2) printf("(%d\t%d)",curr_node, next_node);
			edge_visited_count[curr_node][next_node]= edge_visited_count[curr_node][next_node] + 1;
		}
		else
		{
			// If an edge is passed in same direction twice, loop is said to occur.
			printf("LOOP######## LOOP######## LOOP#######\n");
			printf("source node: %d, dest_node: %d, Failed Link is between %d and %d, prev_node is %d, curr_node is %d and next_node is %d,",source_node, dest_node, node1_with_failed_link, node2_with_failed_link, prev_node, curr_node, next_node);
			printf("Node state is ");
			for(int k=0; k<no_of_nodes; k++)
				printf("%c",buffer[k]);
			//print_ISF(ISF, orig_cost, no_of_nodes, 0);
			//print_ISF(ISF_new, new_cost, no_of_nodes, 1);
            //print_IIF(old_nextHop, no_of_nodes, 0);
		    //print_IIF(new_nextHop, no_of_nodes, 1);
			printf("\n");
			return 1;
		}	
	}
    if(DEBUG_2) printf("\n");
	return 0;
}

void validate_loops_no_print(int node1_for_failed_link, int node2_for_failed_link)
{
	if(DEBUG_2) printf("validate_loops");
	
	/*
	 * Calculate IIF entries for the old topology. 
	 * Nodes adjacent to the failure should use this tables/entries
	 * While other nodes use ISF or ISF_new
	 */
	int old_dist[MxSz];
    for(int vertex = 0;vertex < no_of_nodes; vertex++)
    {
		dij(no_of_nodes, vertex, old_cost, old_dist, old_nextHop);
    }
    
    /*
	 * Calculate IIF entries for the new topology. 
	 * Nodes adjacent to the failure should use this tables/entries
	 * While other nodes use ISF or ISF_new
	 */
    int cost_for_failed_link = new_cost[node1_for_failed_link][node2_for_failed_link];
	
	new_cost[node1_for_failed_link][node2_for_failed_link] = infinity;
	int new_dist[MxSz];
    for(int vertex = 0;vertex < no_of_nodes; vertex++)
    {
		dij(no_of_nodes, vertex, new_cost, new_dist, new_nextHop);
    }
	new_cost[node1_for_failed_link][node2_for_failed_link] = cost_for_failed_link;

    /*
    print_ISF(ISF, orig_cost, no_of_nodes, 0);
	print_ISF(ISF_new, new_cost, no_of_nodes, 1);
    print_IIF(old_nextHop, no_of_nodes, 0);
	print_IIF(new_nextHop, no_of_nodes, 1);
    */
	
	long max = 1 << no_of_nodes;
	for (long i = 0; i < max; i++) 
	{
		long currentNumber = i;

		int bufferPosition = no_of_nodes;
		while (bufferPosition > 0) 
		{
			buffer[--bufferPosition] = (char) (48 + (currentNumber & 1));
			currentNumber = currentNumber >> 1;
		}
        if(DEBUG_2) printf("Buffer: %s", buffer);
		// Process for every element in the buffer array (for each combination, if there can be a loop in the graph
		
		// For one source node
		for(int source_node = 0; source_node < no_of_nodes; source_node++)        
		{
			// Check for all destionation nodes, one by one
			for(int dest_node = 0; dest_node < no_of_nodes; dest_node++)        
			{
				if(source_node == dest_node)
				{
					continue;
				}			
				// If packet can be forwarded to dest_node from source_node
				/* Each node can be in a different state i.e., ISF or ISF_new. 
				 * Go through all combinations one by one
				 */
			
				//int isLoop = check_loop(buffer, source_node, dest_node, node1_for_failed_link, node2_for_failed_link);
				int isLoop = check_loop(source_node, dest_node, node1_for_failed_link, node2_for_failed_link);
				//if(isLoop >0)
				//	return;
			}
		}
	}
}

void validate_loops(int node1_for_failed_link, int node2_for_failed_link)
{
	if(DEBUG_2) printf("validate_loops");
	
	/*
	 * Calculate IIF entries for the old topology. 
	 * Nodes adjacent to the failure should use this tables/entries
	 * While other nodes use ISF or ISF_new
	 */
	int old_dist[MxSz];
    for(int vertex = 0;vertex < no_of_nodes; vertex++)
    {
		dij(no_of_nodes, vertex, old_cost, old_dist, old_nextHop);
    }
    
    /*
	 * Calculate IIF entries for the new topology. 
	 * Nodes adjacent to the failure should use this tables/entries
	 * While other nodes use ISF or ISF_new
	 */
    int cost_for_failed_link = new_cost[node1_for_failed_link][node2_for_failed_link];
	
	new_cost[node1_for_failed_link][node2_for_failed_link] = infinity;
	int new_dist[MxSz];
    for(int vertex = 0;vertex < no_of_nodes; vertex++)
    {
		dij(no_of_nodes, vertex, new_cost, new_dist, new_nextHop);
    }
	new_cost[node1_for_failed_link][node2_for_failed_link] = cost_for_failed_link;

    print_ISF(ISF, orig_cost, no_of_nodes, 0);
	print_ISF(ISF_new, new_cost, no_of_nodes, 1);
    print_IIF(old_nextHop, no_of_nodes, 0);
	print_IIF(new_nextHop, no_of_nodes, 1);
	
	long max = 1 << no_of_nodes;
	for (long i = 0; i < max; i++) 
	{
		long currentNumber = i;

		int bufferPosition = no_of_nodes;
		while (bufferPosition > 0) 
		{
			buffer[--bufferPosition] = (char) (48 + (currentNumber & 1));
			currentNumber = currentNumber >> 1;
		}
        if(DEBUG_2) printf("Buffer: %s", buffer);
		// Process for every element in the buffer array (for each combination, if there can be a loop in the graph
		
		// For one source node
		for(int source_node = 0; source_node < no_of_nodes; source_node++)        
		{
			// Check for all destionation nodes, one by one
			for(int dest_node = 0; dest_node < no_of_nodes; dest_node++)        
			{
				if(source_node == dest_node)
				{
					continue;
				}			
				// If packet can be forwarded to dest_node from source_node
				/* Each node can be in a different state i.e., ISF or ISF_new. 
				 * Go through all combinations one by one
				 */
			
				//int isLoop = check_loop(buffer, source_node, dest_node, node1_for_failed_link, node2_for_failed_link);
				int isLoop = check_loop(source_node, dest_node, node1_for_failed_link, node2_for_failed_link);
				//if(isLoop >0)
				//	return;
			}
		}
	}
}

//Calculates the dead weight for a link i.e., cost of the link, which is equivalent to the link being non-existent (basically value infinity for that link cost).
int get_dead_weight(int i, int j)
{
    short actual_cost[MxSz][MxSz];
    memcpy(actual_cost, orig_cost,  MxSz*MxSz*sizeof(short));
    int dist[MxSz];
    short NH[MxSz][MxSz];
    
    actual_cost[i][j] = infinity;
    actual_cost[j][i] = infinity;
    // From node i calculate next hops for every possible destination
    dij(no_of_nodes, i, actual_cost, dist, NH);
    
    //dist contains the actual cost of edge traversal between i and j, without using i-->j.
    return dist[j];
    
    //return max_edge_cost *2;
    // default value
}

void fifr_increasing_weights() 
{
    // We dont have to consider subset of edges, as we consider single link failure, and not multiple link failures.
    for(int i=0; i<no_of_nodes; i++)
    {
		for(int j=0; j<no_of_nodes; j++)
		{
			if(orig_cost[i][j] > 0 && orig_cost[i][j] != infinity)
			{
				if (i == j)
					continue;

                int dead_weight = get_dead_weight(i, j);
                //if (dead_weight == infinity)
                //    continue;
                
				if(DEBUG_1) printf("Failed link is between %d and %d with dead_weight as: %d \t", i, j, dead_weight);
                
                reset_keylinks(no_of_nodes);
                                
                for(int edge_cost_now = orig_cost[i][j]; edge_cost_now < max_edge_cost *2; edge_cost_now++)
                //for(int edge_cost_now = orig_cost[i][j]; edge_cost_now < dead_weight+1; edge_cost_now++)
                {
                    if(DEBUG_1) printf("Checking for cost between edges %d and %d with cost: %d and %d, with dead_weight as: %d \n", i, j, edge_cost_now, edge_cost_now+1, dead_weight);
                    /*##############################################################################
                     * Calculate the ISF entries for the original (old) topology 
                     * ############################################################################## 
                     */
                    reset_2Darray(old_cost, infinity);    
                    memcpy(old_cost, orig_cost,  MxSz*MxSz*sizeof(short));
                    old_cost[j][i] = edge_cost_now;
                    old_cost[i][j] = edge_cost_now; 
                    reset_3Darray(ISF, infinity);
                    get_isf(old_cost, no_of_nodes, ISF);
                    // ISF entries are calculated
                    if(DEBUG_2) printf("##################################################################");
                    /* ##############################################################################
                     */

                    /* ##############################################################################
                     * Modify the topology by changing one of the *failed* nodes cost to be improved by 1.
                     * Calculate ISF_new entries for the entire topology
                     * ##############################################################################
                     */
                    reset_2Darray(new_cost, infinity);    
                    memcpy(new_cost, orig_cost,  MxSz*MxSz*sizeof(short));
                    new_cost[j][i] = edge_cost_now + 1;
                    new_cost[i][j] = edge_cost_now + 1; 
                    reset_3Darray(ISF_new, infinity);
                    get_isf(new_cost, no_of_nodes, ISF_new);
                    
                    if(DEBUG_2) printf("##################################################################");
                    /* ISF_new entries are calculated after updating cost of one edge.
                     * ##############################################################################
                     */

                    /* Calculate if a loop can be possible with ISF and ISF_new. i and j will follow IIF (i.e., according to dijkstra's)
                     * ##############################################################################
                     */
                    //validate_loops(no_of_nodes, orig_cost, new_cost, ISF, ISF_new, i, j);
                    //validate_loops(no_of_nodes, cost, new_cost, ISF, ISF_new, i, j);
                    validate_loops(i, j);
                    /* Exit
                     * ##############################################################################
                     */
                }		
			}
		}
	}
}

void fifr_increasing_weights_rms() 
{
    // We dont have to consider subset of edges, as we consider single link failure, and not multiple link failures.
    for(int i=0; i<no_of_nodes; i++)
    {
		for(int j=0; j<no_of_nodes; j++)
		{
			if(orig_cost[i][j] > 0 && orig_cost[i][j] != infinity)
			{
				if (i == j)
					continue;

                int dead_weight = get_dead_weight(i, j);
                //if (dead_weight == infinity)
                //    continue;
                
				if(DEBUG_1) printf("Failed link is between %d and %d with dead_weight as: %d \t", i, j, dead_weight);
                
                reset_keylinks(no_of_nodes);

                printf("calling make_route_metric_sequence with weight as: %d\t", orig_cost[i][j]);
                make_route_metric_sequence(i, j, 2 *get_dead_weight(i, j));
                printf("Route metric sequence for link %c-->%c is: \t", i+'A', j+'A');
                
                int edge_cost_old = get_nextmetric();
                for (int edge_cost_now = get_nextmetric(); edge_cost_now != -1; edge_cost_now = get_nextmetric() )
                //for(int edge_cost_now = orig_cost[i][j]; edge_cost_now < max_edge_cost *2; edge_cost_now++)
                //for(int edge_cost_now = orig_cost[i][j]; edge_cost_now < dead_weight+1; edge_cost_now++)
                {
                    if(DEBUG_1) printf("Checking for cost between edges %d and %d with cost: %d and %d\n", i, j, edge_cost_old, edge_cost_now);
                    /*##############################################################################
                     * Calculate the ISF entries for the original (old) topology 
                     * ############################################################################## 
                     */
                    reset_2Darray(old_cost, infinity);    
                    memcpy(old_cost, orig_cost,  MxSz*MxSz*sizeof(int));
                    old_cost[j][i] = edge_cost_old;
                    old_cost[i][j] = edge_cost_old; 
                    reset_3Darray(ISF, infinity);
                    get_isf(old_cost, no_of_nodes, ISF);
                    // ISF entries are calculated
                    if(DEBUG_2) printf("##################################################################");
                    /* ##############################################################################
                     */

                    /* ##############################################################################
                     * Modify the topology by changing one of the *failed* nodes cost to be improved by 1.
                     * Calculate ISF_new entries for the entire topology
                     * ##############################################################################
                     */
                    reset_2Darray(new_cost, infinity);    
                    memcpy(new_cost, orig_cost,  MxSz*MxSz*sizeof(int));
                    new_cost[j][i] = edge_cost_now;
                    new_cost[i][j] = edge_cost_now; 
                    reset_3Darray(ISF_new, infinity);
                    get_isf(new_cost, no_of_nodes, ISF_new);
                    
                    if(DEBUG_2) printf("##################################################################");
                    /* ISF_new entries are calculated after updating cost of one edge.
                     * ##############################################################################
                     */

                    /* Calculate if a loop can be possible with ISF and ISF_new. i and j will follow IIF (i.e., according to dijkstra's)
                     * ##############################################################################
                     */
                    //validate_loops(no_of_nodes, orig_cost, new_cost, ISF, ISF_new, i, j);
                    //validate_loops(no_of_nodes, cost, new_cost, ISF, ISF_new, i, j);
                    validate_loops(i, j);
                    /* Exit
                     * ##############################################################################
                     */
                    edge_cost_old = edge_cost_now;
                }		
			}
		}
	}
}


void print_keylinks_with_increasing_edge_weights(int node1, int node2) 
{
    edge_node1 = node1;
    edge_node2 = node2;
    
    // We dont have to consider subset of edges, as we consider single link failure, and not multiple link failures.
    for(int i=0; i<no_of_nodes; i++)
    {
		for(int j=0; j<no_of_nodes; j++)
		{
			if(orig_cost[i][j] > 0 && orig_cost[i][j] != infinity)
			{
				if (i == j)
					continue;

                int dead_weight = get_dead_weight(i, j);
                //if (dead_weight == infinity)
                //    continue;
                
				if(DEBUG_1) printf("Failed link is between %d and %d with dead_weight as: %d \t", i, j, dead_weight);
                
                reset_keylinks(no_of_nodes);
                                
                //for(int edge_cost_now = orig_cost[i][j]; edge_cost_now < max_edge_cost *2; edge_cost_now++)
                for(int edge_cost_now = orig_cost[i][j]; edge_cost_now < dead_weight+1; edge_cost_now++)
                {
                    if(DEBUG_1) printf("Checking for cost between edges %d and %d with cost: %d and %d, with dead_weight as: %d \n", i, j, edge_cost_now, edge_cost_now+1, dead_weight);
                    /*##############################################################################
                     * Calculate the ISF entries for the original (old) topology 
                     * ############################################################################## 
                     */
                    reset_2Darray(old_cost, infinity);    
                    memcpy(old_cost, orig_cost,  MxSz*MxSz*sizeof(int));
                    old_cost[j][i] = edge_cost_now;
                    old_cost[i][j] = edge_cost_now; 
                    reset_3Darray(ISF, infinity);
                    get_isf(old_cost, no_of_nodes, ISF);
                    // ISF entries are calculated
                    if(DEBUG_2) printf("##################################################################");
                    /* ##############################################################################
                     */

                    /* ##############################################################################
                     * Modify the topology by changing one of the *failed* nodes cost to be improved by 1.
                     * Calculate ISF_new entries for the entire topology
                     * ##############################################################################
                     */
                    reset_2Darray(new_cost, infinity);    
                    memcpy(new_cost, orig_cost,  MxSz*MxSz*sizeof(int));
                    new_cost[j][i] = edge_cost_now + 1;
                    new_cost[i][j] = edge_cost_now + 1; 
                    reset_3Darray(ISF_new, infinity);
                    get_isf(new_cost, no_of_nodes, ISF_new);
                    
                    if(DEBUG_2) printf("##################################################################");
                    /* ISF_new entries are calculated after updating cost of one edge.
                     * ##############################################################################
                     */

                    /* Calculate if a loop can be possible with ISF and ISF_new. i and j will follow IIF (i.e., according to dijkstra's)
                     * ##############################################################################
                     */
                    validate_loops_no_print(i, j);
                    /* Exit
                     * ##############################################################################
                     */
                }		
			}
		}
	}
}

int main (int argc, char *argv[]) 
{
    int opt = 0;
    char *in_fname = NULL;
    int set_ISF_print = 0;
    int validate = 0;
    int trace_keylinks = 0;
    int createrms = 0;

    // Read command line parameters correctly
    while ( (opt = getopt(argc, argv, "f:p:v:n:r:")) != -1) 
    {
        switch(opt) 
        {
            case 'f':
                in_fname = optarg;
                printf("\nConsidered topology name is%s", in_fname);
                break;
            case 'p':
                set_ISF_print = atoi(optarg);
                printf("\nOption to print ISF tables is set=%d", set_ISF_print);
                break;
            case 'v':
                validate = atoi(optarg);
                printf("\nOption :validate if loops are possible: is set=%d", validate);
                break;
            case 'n':
                trace_keylinks = atoi(optarg);
                printf("\nOption :Trace Keylinks for each link separately: is set=%d", trace_keylinks);
                break;
            case 'r':
                createrms = atoi(optarg);
                printf("\nOption :create route metric sequence for each link separately: is set=%d", createrms);
                break;
            case '?':
                /* Case when user enters the command as
                 */
                if (optopt == 'f') 
                {
                    printf("\nMissing input topology File option");
                    /* Case when user enters the command as
                     * # ./validate -f
                     */
                     return 0;
                } else if (optopt == 'p') 
                {
                    printf("\nMissing mandatory printISF_IIF tables option");
                    return 0;
                } else if (optopt == 'v') 
                {
                    printf("\nMissing mandatory validate_for_loops option");
                    return 0;
                } else if (optopt == 'n') 
                {
                    printf("\nMissing mandatory trace_keylinks option");
                    return 0;
                } else 
                {
                     printf("\nInvalid option received");
                     return 0;
                }
                break;
        }
    }

    printf("\n");

	/*##############################################################################
	 *
	 * Fetch the original topology
	 *
	 * First line of the file contains number of nodes in the graph 
	 * Reads input until -1 is seen. This is notation used to determine the end of file
	 */
    FILE *fp;
    fp = fopen(in_fname, "r");    
    if (fp == NULL)
    {
        printf("Invalid file \n");
        return 0;
    }
    
    printf("Reading topology from file \n");
	int i = 0, j = 0, edges = 0;
	short cost[MxSz][MxSz] = {0};
	reset_2Darray(cost, infinity);
	reset_2Darray(orig_cost, infinity);
	
	fscanf(fp, "%d", &no_of_nodes);
	fgetc(fp);
	while(1)
	{	
		fscanf(fp, "%d", &i);
		getc(fp);
		if(i == -1)
			break;
		fscanf(fp, "%d", &j);
		getc(fp);
		if(j == -1)
			break;
		edges++;
		fscanf(fp, "%hu", &cost[i][j]);

		cost[j][i] = cost[i][j];
        if(max_edge_cost < cost[j][i])
            max_edge_cost = cost[j][i];
		getc(fp);			
	}

	memcpy(orig_cost, cost,  MxSz*MxSz*sizeof(int));
    
    fclose(fp);            printf("Reading topology from file is COMPLETED \n");
    /* Topology is read from the file correctly.
	 * ##############################################################################
	 */

    // Perform operations accordingly.
    if (set_ISF_print == 1)
    {
        get_isf(cost, no_of_nodes, ISF);
        print_ISF(ISF, cost, no_of_nodes, 0);

        int old_dist[MxSz];
        for(int vertex = 0; vertex < no_of_nodes; vertex++)
        {
           dij(no_of_nodes, vertex, cost, old_dist, old_nextHop);
        }
        print_IIF(old_nextHop, no_of_nodes, 0);

    }
    else if (validate == 1 && createrms == 1)
    {
        printf("Validating if loops are possible in this topology with route metric sequences \n");
        fifr_increasing_weights_rms();
    }
    else if (validate == 1)
    {
        printf("Validating if loops are possible in this topology \n");
        fifr_increasing_weights();
    }
    else if (trace_keylinks == 1)
    {
        for (int i=0; i< no_of_nodes; i++)
        {
            for (int j=0; j<no_of_nodes; j++)
            {
                if (cost[i][j] > 0)
                    print_keylinks_with_increasing_edge_weights(i, j);
            }
        }
        
    }
    else if (createrms == 1)
    {
        for (int i=0; i< no_of_nodes; i++)
        {
            for (int j=0; j<no_of_nodes; j++)
            {
                if (i != j && orig_cost[i][j] != infinity)
                //if (cost[i][j] > 0)
                {
                    printf("calling make_route_metric_sequence with weight as: %d\t", cost[i][j]);
                    make_route_metric_sequence(i, j, 2 *get_dead_weight(i, j));
                    printf("Route metric sequence for link %c-->%c is: \t", i+'A', j+'A');
                    for (int k = get_nextmetric(); k != -1; k = get_nextmetric() )
                    {
                        printf("%d\t", k);
                    }
                    printf("\n");
                }
            }
        }
    }

    return 0;
}
