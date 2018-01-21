//This code implements the logic to calculate ISF table entries at each router, for a given topology. 

#include "get_isf.h"

void print_kl_v2(int KL[MxSz][MxSz][MxSz], int n, int SubTree[MxSz])
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
					if(KL[i][j][k] == 1)
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

void print_IIF(const int IIF[MxSz][MxSz], const int n, int state)
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


void print_ISF(const int ISF[MxSz][MxSz][MxSz], const int cost[MxSz][MxSz], const int n, const int state)
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
void reset_3Darray(int KL[MxSz][MxSz][MxSz], int reset)
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
void reset_2Darray(int NH[MxSz][MxSz], int reset)
{
	int i,j;
	for(i = 0; i < MxSz; i++)
		for(j = 0; j < MxSz; j++)
			NH[i][j] = reset;
}
void reset_1Darray(int array[MxSz], int reset)
{
	int i;
	for(i=0 ; i < MxSz; i++)
		array[i] = reset;
}

/* Calculate minimum costs and next hops according to Dijkstra's algorithm */
void dij(int n, int v, const int cost[MxSz][MxSz], int dist[], int NextHop[MxSz][MxSz])
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
void makeISF(int KL[MxSz][MxSz][MxSz], int cost[MxSz][MxSz], int dist[MxSz], int SubTree[MxSz], int n, int i, int j, int ISF[MxSz][MxSz][MxSz])
{
    // i and j reference j->i interface in which these key links apply
    int u,v, d, d2; // KL is u v d where u and v are the edge which is a keylink to d marked as 1
    int NH[MxSz][MxSz], orig_cost[MxSz][MxSz];
    
    memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
    for(d = 0; d < n; d ++)
	{
		reset_2Darray(NH, infinity);
        if(SubTree[d] == 1)
        {
			for(u = 0; u < n; u++)
	        {
		        for( v = 0; v < n; v++)
		        {
					if(KL[u][v][d] == 1)
                    {
                    //     printf("\nFor Destination %c the link ", d +'A');
		            //   	printf("%c -> %c is removed\n", u + 'A', v + 'A');
						cost[u][v] = infinity;
                        cost[v][u] = infinity;
					}
		        }
	        }
            //  print_cost(cost,n);
            dij(n, i, cost, dist, NH);
            //  printf("\n\n<<<<<<<<<<<<< DEST NH >>>>>>>>>>>>>\n\n");
            //  print_Dest_NextHop(NH, n);

            memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
			d2 =d;
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
	memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));

	//printf("ISF %c->%c to %c is %c\n", j + 'A', i + 'A', d+ 'A', ISF[i][j][d]+ 'A' );
    //CAN we "Smartly" compute this like the print_kl? or use the fact we just built it ?
}

void makeSPT(int source, int dest, int NextHop[MxSz][MxSz], int SPT[MxSz][MxSz])
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

void makeSubTree_v2(int subvertex, int SPT[MxSz][MxSz], int n, int SubTree[MxSz])
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

// ORIGNAL ISF from PAPER
void runISF_orig(int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz])
{
    int dist[MxSz], NextHop[MxSz][MxSz], origNextHop[MxSz][MxSz], SPT[MxSz][MxSz];
	int orig_cost[MxSz][MxSz]; // Store original costs
	int i, j, u, v, k, m, d; 

	int SubTree[MxSz];
	int SubTree2[MxSz];

	int KeyLinks[MxSz][MxSz][MxSz];

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

	memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
	memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));

	for(i = 0; i < n; i++)
	{
		// printf("\n\n******\t******\t******\t******\tKey Links for %c\t******\t******\t******\t******\n",i +'A');
		memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
		memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
		for(j = 0;j < n; j ++)
		{
			if(i == j)
				continue;
			if(cost[i][j] == infinity)
				continue;
			memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
			memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
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
						// printf("HERE %dgeany , %d\n", u, v);
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
										KeyLinks[u][v][d] = 1;
									}
								}
							}
							reset_1Darray(SubTree2,0);
					 }
					memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
				}
				memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
			}

			if (DEBUG_KL) printf("\nThe Interface from %c->%c\n", j +'A', i + 'A');
            if (DEBUG_KL) print_kl_v2(KeyLinks, n, SubTree);

			// Compute ISF F^d j->i = R^d i (all edges excluding/KeyLinks^d j->i)
			makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF);
			memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
			reset_3Darray(KeyLinks, infinity);
		}
	}
}

//Print the edge costs taken from the input file (In other words: used to verify if the input is same as what is given) 
void print_cost(int cost[MxSz][MxSz], int n)
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
void get_isf(int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz])
{
	if(DEBUG_2) print_cost(cost, n);
	runISF_orig(cost, n, ISF);
	if(DEBUG_2) print_ISF(ISF, cost, n, 0);
    if(DEBUG_2) printf("\n"); 
}
