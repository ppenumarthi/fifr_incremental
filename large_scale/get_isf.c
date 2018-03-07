//This code implements the logic to calculate ISF table entries at each router, for a given topology. 

#include "get_isf.h"

short old_KeyLinks[MxSz][MxSz][MxSz][MxSz][MxSz] = {0};
short new_KeyLinks[MxSz][MxSz][MxSz][MxSz][MxSz] = {0};

extern int edge_node1;
extern int edge_node2;

void reset_keylinks(int no_of_nodes)
{
    int n=no_of_nodes;
    
    for(short i=0;i < n;i++)
	{
		for(short j=0;j < n;j++)
		{
			for(short k=0; k < n; k++)
            {
                for(short l=0; l < n; l++)
                {
                    for(short m=0; m < n; m++)
                    {
                        old_KeyLinks[i][j][k][l][m]= 0;
                        new_KeyLinks[i][j][k][l][m]= 0;
                    }
                }
            }
	     }
	}
}

void print_kl_v2(short KL[MxSz][MxSz][MxSz], int n, short SubTree[MxSz])
{
	int i = 0, j = 0, k = 0;
	for(k = 0; k < n; k ++)
	{
		if(SubTree[k] == 1)
        {
			for(i = 0; i < n; i ++)
	        {
		        for( j = 0; j < n; j ++)
		        {
					//if ( (KL[i][j][k] == 1) && ( i == edge_node1 && j == edge_node2) )
					if ( (KL[i][j][k] == 1) )
                    {
                        printf("\nFor Destination %c the link ", k +'A');
		                printf("%c -> %c is a Key link\n", i + 'A', j + 'A');
                    }
		        }
	        }
         }
         else
			continue;
	}
}

void print_IIF(const short IIF[MxSz][MxSz], const int n, int state)
{
    int i,j,d;

    if (state == 0)
        printf("\n\t\tIIF_OLD TABLE \n");
    else if (state == 1)
        printf("\n\t\tIIF_NEW TABLE \n");
	else if (state == 2)
	    printf("\n\t\tIIF_NEW_WITH_BROKEN_LINK_COST_INFINITY TABLE \n");

    printf("\n");
    printf("\nsrc/des |");
        
    for( j = 0; j <n; j++)
        printf(" %c |", j + 'A');

    printf("\n--------");
    for( j = 0; j <n; j++)
        printf("----");

    for( i = 0; i < n; i ++)
    {
        printf("\n\t");     
        for( j = 0; j <n; j++)
            printf(" %c |", IIF[i][j] + 'A');
   
    }
}


void print_ISF(const short ISF[MxSz][MxSz][MxSz], const short cost[MxSz][MxSz], const int n, const int state)
{
    int i,j,d;
    // printf("\n<<<<<<<<<<<<<<<<<<<<<\tInterface Specific Forwarding Tables\t>>>>>>>>>>>>>>>>>>>>>\n");
    if (state == 0)
        printf("\n\t\tISF_OLD TABLE \n");
    else
        printf("\n\t\tISF_NEW TABLE \n");
    for( i = 0; i < n; i ++)
    {
        printf("\n");
        printf("\nIF/des |");
        for( j = 0; j <n; j++)
            printf(" %c |", j + 'A');
        printf("\n--------");
        for( j = 0; j <n; j++)
            printf("----");
        for( j = 0; j < n; j ++)
        {
            if(j == i || cost[i][j] == infinity)
                continue;
            else
            {
                printf("\n%c -> %c $", j + 'A', i + 'A');
                for(d=0;d < n; d++)
                {
                    if(ISF[i][j][d] < infinity && j != d && j != ISF[i][j][d])
                        printf(" %c |", ISF[i][j][d] + 'A');
                    else if(j == ISF[i][j][d])
                        printf(" %c*|", ISF[i][j][d] + 'A');
                    else
                        printf(" - |");
                }
            }
		}
    }
}

/* Equivalent to memset for multiple dimensional arrays */
void reset_5Darray(short KL[MxSz][MxSz][MxSz][MxSz][MxSz], int reset)
{
	int i, j, k, l, m;
	int n = MxSz;
    
	for(i=0;i < n;i++)
	{
		for(j=0;j < n;j++)
		{
			for(k=0; k < n; k++)
            {
                for(l=0; l < n; l++)
                {
                    for(m=0; m < n; m++)
                    {
                        KL[i][j][k][l][m]= reset;
                    }
                }
            }
	     }
	}
}

void reset_3Darray(short KL[MxSz][MxSz][MxSz], int reset)
{
	int i, j, k;
	int n = MxSz;
	for(i=0;i < n;i++)
	{
		for(j=0;j < n;j++)
		{
			for(k=0; k < n; k++)
				KL[i][j][k]= reset;
	     }
	}
}

void reset_2Darray(short NH[MxSz][MxSz], int reset)
{
	int i,j;
	for(i = 0; i < MxSz; i++)
		for(j = 0; j < MxSz; j++)
			NH[i][j] = reset;
}

void reset_1Darray(short array[MxSz], int reset)
{
	int i;
	for(i=0 ; i < MxSz; i++)
		array[i] = reset;
}

void reset_1Darray(int array[MxSz], int reset)
{
	int i;
	for(i=0 ; i < MxSz; i++)
		array[i] = reset;
}

/* Calculate minimum costs and next hops according to Dijkstra's algorithm */
void dij(int n, int v, const short cost[MxSz][MxSz], int dist[], short NextHop[MxSz][MxSz])
{
	 int i,u,count,w,flag[MxSz],min;
	 for(i=0;i < n;i++)
	 {
		flag[i]=0;
		dist[i]=cost[v][i];
		//printf("Initial Cost from vertex %c to dest %c is %d\n", v + 'A', i + 'A', dist[i]);
		if(dist[i] < infinity)
		{
			// Setting Next hop array (initialized as the only connected nodes)
			NextHop[v][i]= i;
		}
		else
		{
			//NextHop[v][i] = infinity;
		}
	 }

	 count=2;
	 while(count<n)
	 {
        // printf("HERE in dij, v: %d, u: %d\n", v, u);
		min=infinity - 1;
		//printf("Vertex in Dij is %c\n", v + 'A');
		//print_cost(cost, n);
		for( w=0; w<n; w++)
		{
			if(dist[w]<min && !flag[w])
			{
				min=dist[w]; 
				u=w;
			}
		}
		if( u < n && u >= 0)
			flag[u]=1;
		else
		{
			/*printf("\n Graph is not connected with vertex %c\n", v + 'A'); 
			 * print_cost(cost, n)*/; 
			return;
		}
		count++;
		//printf("HERE in dij, v: %d\n", v);
		for(w=0;w<n;w++) 
		{
			if(((dist[u]+cost[u][w]<dist[w]) && !flag[w]) && v != w)
			{
				dist[w]=dist[u]+cost[u][w];
				// builds next hop from initialized (will build with only those)
				NextHop[v][w] = NextHop[v][u];
				//printf("Vertex(v) is %c, Next Hop[v][u] is %c, for Destination(w) is %c, u is %c \n",  v + 'A', NextHop[v][u]+ 'A', w + 'A', u + 'A');
			}
			else if (((dist[u]+cost[u][w]==dist[w]) && !flag[w]) && v != w)
			{
				if( NextHop[v][u] < NextHop[v][w])
					NextHop[v][w] = NextHop[v][u];
			}
			else
			{
				//printf("NH from vertex %c to dest %c is %c\n", v + 'A', w + 'A', NextHop[v][w] +'A');
			}
		}
	 }
}

/*
    Takes an ISF table and using the KL matrix, removes them from the cost matrix to compute the ISF table for all d on j->i
*/
void makeISF(short KL[MxSz][MxSz][MxSz], short cost[MxSz][MxSz], int dist[MxSz], short SubTree[MxSz], int n, int i, int j, short ISF[MxSz][MxSz][MxSz])
{
    // i and j reference j->i interface in which these key links apply
    int u,v, d, d2; // KL is u v d where u and v are the edge which is a keylink to d marked as 1
    short NH[MxSz][MxSz], orig_cost[MxSz][MxSz];
    
    memcpy(orig_cost, cost, MxSz*MxSz*sizeof(short));
    for(d = 0; d < n; d ++)
	{
		reset_2Darray(NH, infinity);
        if(SubTree[d] == 1)
        {
			for(u = 0; u < n; u++)
	        {
		        for( v = 0; v < n; v++)
		        {
                    /* */
                    if(KL[u][v][d] == 1)
                    {
                    //     printf("\nFor Destination %c the link ", d +'A');
		            //   	printf("%c -> %c is removed\n", u + 'A', v + 'A');
						cost[u][v] = infinity;
                        cost[v][u] = infinity;
					}
                    /* */
		        }
	        }
            //  print_cost(cost,n);
            dij(n, i, cost, dist, NH);
            //  printf("\n\n<<<<<<<<<<<<< DEST NH >>>>>>>>>>>>>\n\n");
            //  print_Dest_NextHop(NH, n);

            memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));
			d2 = d;
			ISF[i][j][d2] = NH[i][d2];
       }
       else
       {
			// There we no key links compute NH "regularly"
            dij(n, i, cost, dist, NH);
            ISF[i][j][d] = NH[i][d];
            //continue;
       }
	}
	memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));

	//printf("ISF %c->%c to %c is %c\n", j + 'A', i + 'A', d+ 'A', ISF[i][j][d]+ 'A' );
    //CAN we "Smartly" compute this like the print_kl? or use the fact we just built it ?
}

void makeSPT(int source, int dest, short NextHop[MxSz][MxSz], short SPT[MxSz][MxSz])
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

void makeSubTree_v2(int subvertex, short SPT[MxSz][MxSz], int n, short SubTree[MxSz])
{
	int m;
    for(m = 0; m < n; m ++)
	{
		// If needs to exclude node checking itself 0 will be another variable?
		if(SPT[subvertex][m] == 1) // Looks through all of A's next hops to see if matches j.
		{
			// Get subtree below j, rooted at A
			// Flag all nodes that use j as next hop, V Prime (should be all of j's subtree)
			SubTree[m] = 1;
            makeSubTree_v2(m, SPT, n, SubTree);
		}
	}
}

int getdist(int no_of_nodes, int src, int dest, short cost[MxSz][MxSz])
{
    if (src == dest)
        return 0;
    int dist[MxSz];
    short NextHop[MxSz][MxSz];
    reset_2Darray(NextHop, infinity);
    reset_1Darray(dist, infinity);        
    dij(no_of_nodes, src, cost, dist, NextHop);
    return dist[dest];
}

int isEmpty(short KL[MxSz][MxSz][MxSz], int curr_dest, int no_of_nodes)
{
    for(int i=0; i < no_of_nodes; i++)
    {
        for(int j=0; j < no_of_nodes; j++)
        {
            if( KL[i][j][curr_dest] != infinity)
                return 0;
        }        
    }
    return 1;
}

// ORIGNAL ISF from PAPER
void runISF_orig(short cost[MxSz][MxSz], int n, short ISF[MxSz][MxSz][MxSz])
{
    int dist[MxSz];
    short NextHop[MxSz][MxSz], origNextHop[MxSz][MxSz], SPT[MxSz][MxSz];
	short orig_cost[MxSz][MxSz]; // Store original costs
	int i, j, u, v, k, m, d; 

	short SubTree[MxSz];
	short SubTree2[MxSz];

	short KeyLinks[MxSz][MxSz][MxSz];

    reset_1Darray(SubTree, 0);
    reset_1Darray(SubTree2, 0);
    reset_2Darray(SPT, infinity);
    reset_2Darray(NextHop, infinity);
    reset_3Darray(KeyLinks, infinity);

	// We compute Next Hop and store it in an array to keep original.
	for(v = 0;v < n; v++)
	{
		dij(n, v, cost, dist, NextHop);
	}

	memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(short));
	memcpy(orig_cost, cost, MxSz*MxSz*sizeof(short));

	for(i = 0; i < n; i++)
	{
		// printf("\n\n******\t******\t******\t******\tKey Links for %c\t******\t******\t******\t******\n",i +'A'); //current_node
		memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));
		memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(short));
		for(j = 0; j < n; j++) //next_node
		{
			if(i == j)
				continue;
			if(cost[i][j] == infinity)
				continue;
			memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));
			memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(short));
			reset_1Darray(SubTree, 0);
			reset_1Darray(SubTree2, 0);
			reset_2Darray(SPT, infinity);
			// LINE 2 of ALGORITHM KEY LINKS
			reset_3Darray(KeyLinks, infinity);
			// LINE 3 of ALGORITHM KEY LINKS
			// fprintf(stderr, " HERE 2.1  %d %d \n", i, j);
			// print_Dest_NextHop(NextHop, n);
			for(v = 0; v < n; v++)
			{
				//   fprintf(stderr, " HERE 2.10  %d %d NH %d\n", i, v, NextHop[i][v]);
				makeSPT(i, v, NextHop, SPT);
			}
			// fprintf(stderr, " HERE 2.2  %d %d \n", i, j);

			// LINE 4 and 5 of ALGORITHM KEY LINKS
			if(NextHop[i][j] != j)
			{
				makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF);
				continue;
			}
			// We will do for j->i, where j for this will be B


			// LINE 6 of ALGORITHM KEY LINKS
			// Flagging vertices of Subtree
			// Look at all nodes from SPT of i rooted at j for next hop.
			// They are flagged in Subtree
			makeSubTree_v2(j, SPT,n,SubTree);
			// makeSubTree(i, j, NextHop, n, SubTree);
			
			// LINE 7 of ALGORITHM KEY LINKS
			// This will compute SPF without u->v rooted at u for all u->v of E excluding j-> i (b to A) for this
			for( u = 0; u < n; u++)
			{
				for( v = 0; v < n; v++)
				{
					if((v == i && u == j)||(v == j && u == i) || u == v || cost[u][v] == infinity)
						// Exluding (j->i) and links that don't exist
						continue;
					else
					{
						// LINE 8 of ALGORITHM KEY LINKS
						// Removing u->v, Making the cost of link u->v infinity must compute new SPF via dij
						cost[u][v] = infinity;
						cost[v][u] = infinity;
						// printf("HERE %d, %d\n", u, v);
						for(k = 0;k < n; k++)
						{
							// printf("runISF function\n\n");
							dij(n, k, cost, dist, NextHop); }
							// Create a new SPT T^u->v u, From u go through each node (m) to compute edges that make SPT (recursive)
							reset_2Darray(SPT, infinity);
							//   printf("HERE 2\n");
							for(m = 0; m < n; m++)
								makeSPT(u, m, NextHop, SPT);
							
							// LINE 9 of ALGORITHM KEY LINKS
							// if SPT[j][i] (j->i) is == 1, then j->i is an element of the Edges(T^u->v u)
							if(SPT[j][i] == 1)
							{
								makeSubTree_v2(i, SPT,n,SubTree2);

								// Line 10  of ALGORITHM KEY LINKS
								for(d = 0; d < n; d++)
								{
									if((SubTree2[d] == 1 && SubTree[d] == 1))
									// LINE 11 of ALGORITHM KEY LINKS
									{
										//printf("\n Key Link %c - %c for destination %c added to %c->%c\n", u + 'A', v + 'A', d + 'A', j+ 'A', i+ 'A');
										//--> Need to do this
                                        KeyLinks[u][v][d] = 1;
                                        new_KeyLinks[j][i][u][v][d] = 1;
                                        
                                        //Phani Modifications starts
                                        // To make sure there is only one keylink, at any point of time
                                        //Check if there are other keylinks to this destination 'd' for link j-->i
                                        for(int node1 = 0; node1 < n; node1++)
                                        {
                                            for(int node2 = 0; node2 < n; node2++)
                                            {
                                                if (node1 == u && node2 == v )
                                                    continue;
                                                if (KeyLinks[node1][node2][d] == 1)
                                                {
                                                    /* Pick either u-->v or node1-->node2
                                                     */
                                                    int dist1 = getdist(n, v, d, orig_cost);
                                                    int dist2 = getdist(n, node2, d, orig_cost);
                                                    
                                                    //Since we look at u-->v and node1-->node2; we check for distance between v and node2 only.
                                                    if( dist1 < dist2)
                                                    {
                                                        KeyLinks[u][v][d] = 1;
                                                        KeyLinks[node1][node2][d] = 0;
                                                        new_KeyLinks[j][i][u][v][d] = 1;
                                                        new_KeyLinks[j][i][node1][node2][d] = 0; 
                                                    }
                                                    else
                                                    {
                                                        KeyLinks[u][v][d] = 0;
                                                        KeyLinks[node1][node2][d] = 1;
                                                        new_KeyLinks[j][i][u][v][d] = 0;             
                                                        new_KeyLinks[j][i][node1][node2][d] = 1;
                                            
                                                    }
                                                    /* */
                                                }
                                                    
                                            }                                            
                                        }
                                        // Phani modifications end
									}
								}
							}
							reset_1Darray(SubTree2,0);
					 }
					memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));
				}
				memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));
			}

			if (DEBUG_KL) printf("\nThe Interface from %c->%c\n", j +'A', i + 'A');
            if (DEBUG_KL) print_kl_v2(KeyLinks, n, SubTree);

			// Compute ISF F^d j->i = R^d i (all edges excluding/KeyLinks^d j->i)
            /* Phani modifications start
             * Check if KL_new is empty
             * If it is empty, use KL_old
             * else use KL_new
             * For each destination, verify if keylinks exist: if they are empty, copy form old_keylinks.
             */
            
            for (int curr_dest = 0; curr_dest < n; curr_dest++)
            {
                if ( isEmpty(KeyLinks, curr_dest, n) == 1)
                {
                    //if (DEBUG_KL) printf("\nUsing Old keyLinks for dest: %c now, as current keylinks are empty", curr_dest+'A');                
                    for(int l=0; l<n; l++)
                    {
                        for (int m=0; m<n; m++)
                        {
                            if (old_KeyLinks[j][i][l][m][curr_dest] == 1)
                            {
                                KeyLinks[l][m][curr_dest] = 1;
                            	if (DEBUG_KL) printf("\nFor dest: %c keylink added as %c->%c\n", curr_dest +'A', l +'A', m + 'A');
                            }
                        }
                    }
                }
            }
            makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF);
           
            /* * /
			// makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF); --Previously this existed.
            /* Phani modifications end
             * */
             
			memcpy(cost, orig_cost, MxSz*MxSz*sizeof(short));
                        
		    reset_3Darray(KeyLinks, infinity);
		}
	}
}

//Print the edge costs taken from the input file (In other words: used to verify if the input is same as what is given) 
void print_cost(short cost[MxSz][MxSz], int n)
{
	int i = 0, j = 0;
	printf("Matrix\nF\\T|");
	for(i = 0; i < n; i ++)
	{
		printf(" %c |", 'A' + i);
	}
	printf("\n----");
	for(i = 0; i < n; i ++)
	{
		printf("----");
	}
	printf("\n");
	for(i = 0; i < n; i ++)
	{
		printf(" %c $", 'A' + i);
		for( j = 0; j < n; j ++)
		{
			if(i == j)
				printf(" - |");
			else if(cost[i][j] == infinity)
				printf(" - |");
			else
				printf(" %d |", cost[i][j]);
		}
		printf("\n");
	}
}

// Function that acts as an anchor to fetch ISF table entries at each router in the topology.
void get_isf(short cost[MxSz][MxSz], int n, short ISF[MxSz][MxSz][MxSz])
{
    for(int a=0; a<MxSz; a++)
    for(int b=0; b<MxSz; b++)
    for(int c=0; c<MxSz; c++)
    for(int d=0; d<MxSz; d++)
    for(int e=0; e<MxSz; e++)
        old_KeyLinks[a][b][c][d][e]= new_KeyLinks[a][b][c][d][e];
        
    reset_5Darray(new_KeyLinks, 0);    
	if(DEBUG_2) print_cost(cost, n);
	runISF_orig(cost, n, ISF);
	if(DEBUG_2) print_ISF(ISF, cost, n, 0);
    if(DEBUG_2) printf("\n"); 
}
