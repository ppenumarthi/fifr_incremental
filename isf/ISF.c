#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

/***********************************************************
* You can use all the programs on  www.c-program-example.com
* for personal and learning purposes. For permissions to use the
* programs for commercial purposes,
* contact info@c-program-example.com
* To find more C programs, do visit www.c-program-example.com
* and browse!
*
*                      Happy Coding
Takes input from file as :
./a.out < test.txt
First integer is # of nodes then,
declare a node A to node B with weight of 4
A-B=4
$ is a "quit node" for inputing connections
Last node is one to compute paths from

4 nodes, with edges of weights, then done link input with a '$'

4
A B 3
A C 3
A D 1
B D 5
C D 1
$

isf9.c trying to make this mirror the algorithm more closely,

isf11.c Key links are outputted correctly for B->A

isf13.c Computes all Key Links, cleaning out code.

isf18c.c ISF_SIM 16 MAY, built from isf18c.c,
       - made original main() into function runISF, for purposes that will run the algorithm on different topos
       - made traverse function

ISF_SIM3 17 MAY
        - Fixed/ added memcpy in traverse (caused big errors in traversing without)
        - working on making another ISF matrix (ISF_t) to hold ISF of topo with down link

ISF_SIM4 17 MAY
        - Created a bit counter to maintain states of nodes to compute various permutations of ISF old vs ISF temp to find if any loops occuer
        - Currently at SIM4, only change is that the starting Nexthop is changed if initial node has new state

ISF_SIM5 17 MAY
        - Removed BWT functions (were not being used in previous iterations), as 16 MAY Prof. stated BWT can just be a re-compute of Dij without link (down), BWT are only needed for router with down edge.
        - Added Graph not connected check in Dij
        - Runs ISF and ISF_temp, with permutations of states
        - It looks as if ISF temp can be routers next to failure retain ISF old state (to utilize BWT properly), and all other routers that move to ISF temp must retain ISF routes which return to same interface it received on. ( ex. A-> B, and A is next hop, B in ISF temp will return to A )

ISF_SIM6 17 MAY
        - Running into problems with topologies where node is no longer connected.

ISF_SIM7a 18 MAY
        - Creating a ISF temp, where DIJ old and DIJ new are compared, the differences are then taken and inserted as the Next Hop for EVERY DESTINATION on each interface in which they were different.

ISF_SIMtest2.c 19 May
        - Added comments detailing traverse, in a if STMT, had a check of state and u == nh/ v ==nh. Removing it did not seem to have an effect on anything. The check did not seem to make sense.
        - Currently the Counter topo still creates loops. All others seem to work.
        - Added output only if loops happen.

ISF_SIMtest3.c 19 May
        - Attempting to implement for ISF old, if the route computed for an ISF is still j, in an instance j->i, then for that destination, will compute a "worse case" scenario, where all links along that path are down...
        - This works for the firmul and counter topos. In PPT wrote details on why this could be a sol'n or on the way to sol'n

ISF_SIMtest3b.c 20 May
        - In main() implemented the ISF old idea of computing complete path failure for ISF entries that return to interface received (no KLs is the assumption here) having issues where all entries are not being computed.

ISF project from ISF_SIMtest3b.c 1 June
        - Assumption now is to start with ISF old and then when an LSA down is received a new, less optimal fowarding can be done. This new FWD table is adding a new
        condition similiar to condtioin 2 for KLs. However it is without u-v, will j -> i be along shortest path from j to d. Originally KL condition 2 was from u to d.
        - This condition does not seem to interfer much with ISF, and may be able to be implemented in an ISF steady state. The idea though is that the node though
        does not know the failure is now expecting that its neighbor may, and therefore must compute a path differently.
        - Now this new state will be ISF temp. The procedure of nodes in OSPF is that it will send LSA updates before it sends any more packets, so it can
        be assumed that the neighbor knows a transition will be taking place.
        - The next step is between ISF temp and ISF temp2. ISF temp2 behaves like IIF new (it knows the failure)

ISF project 2 June, saved as ISF3.c before cleanup
        - It looks as if temp1 and temp2 work together. Temp1 being a ISF old state and temp2 being a modified ISF new state
        - Had weird computations in beginning of ISF to "fill in" table with next hops and then rewrite with ISF. Instead properly
        inserted in the makeISF function on the else to just insert there. (key links empty)
        - Used makeSubTree_v2 function instead of original.
3 June
        - added a bunch more print functions
        - added traverse_steady function which takes an ISF and checks to see if that one state has loops
        - added traverse_t2_stdy, temp2 with steady and checks permutations of nodes in those 2 states for loops
        - added a check in traverse functions if nexthop (nh == infinity) then output not reachable

4 June
        - runISFtemp is required for each "side" of ISF computation. Meaning on the old and new topology condition 2 must be from j
        to d.
        - In Main ran a permutation of the topology read in, and runs the traverse functions on everyy combination of removed links
        in that topology
        - made a timer to time runs
        - traverse_steady breaks out when next hop is not reachable (fixes seg fault)
        - changed variables for ipow function to long long ints for large computations (realize time would be astronomical anyway)
        - Once first Loop is found a "run" exits program

6 June
        - runISFtemp2 is back to "original" idea of IIF new+ ISF old. Also to "handle" ECMP, dijkstra's ensures that if there are equal paths then
        the node that is lower in value is chosen.
        - This method seems to work for all.
        - Also this method employs the KL cond 2 change

8 June
        - changed names of functions to be more relevant. If it starts with z_, function is old and not used.
        - runISF_steady and runISF_temp are only 2 "versions" used
        - traverse_steady_to_temp takes in account traversing in all steay and in all temp (when all states are 0/1)
        - traverse_temp_to_steady is traversing with both after FIB
***********************************************************/

#//include <conio.h>
#define MxSz 9
#define infinity 999

// Run Dijkstras with the total number of nodes, the vertex, initial cost matrix, NextHop gets filled in by the function. distance is empty
void dij(int n,int v,int cost[MxSz][MxSz],int dist[], int NextHop[MxSz][MxSz]);

void print_Dest_NextHop(int NH[MxSz][MxSz], int n);

void print_cost(int NH[MxSz][MxSz], int n);

void print_kl_v2(int KL[MxSz][MxSz][MxSz], int n, int SubTree[MxSz]);

void print_ISF(int ISF[MxSz][MxSz][MxSz], int cost[MxSz][MxSz], int n);

// Runs and Prints out all ISFs
void print_link_down_ISF(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

// Prints all relevant information if a loop occured
void print_loop(int n, int loopOut[MxSz], int lc, int u, int v, int s, int d, int state[MxSz], int ISF[MxSz][MxSz][MxSz], int ISF_t[MxSz][MxSz][MxSz], int ISF_t_orig[MxSz][MxSz][MxSz], int NextHop_t[MxSz][MxSz], int NextHop[MxSz][MxSz], int cost[MxSz][MxSz], int orig_cost[MxSz][MxSz]);

void print_topo(int cost[MxSz][MxSz], int n);
// Resets an arrady of size to the reset value
void reset_1Darray(int array[MxSz], int reset);
void reset_2Darray(int NH[MxSz][MxSz], int reset);
void reset_3Darray(int KL[MxSz][MxSz][MxSz], int reset);

// make a SPT with the source, the destination, and the NextHop
void makeSPT(int source, int dest, int NextHop[MxSz][MxSz], int SPT[MxSz][MxSz]);

// make a SubTree from a vertex given a SPT.
void makeSubTree_v2(int subvertex, int SPT[MxSz][MxSz], int n, int SubTree[MxSz]);

// Runs the Key Link Algorithm and calls Ŕ to fill in the ISF
void runISF_orig(int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz]);

// Takes the Key Links and the subtree and fills in the ISF
void makeISF(int KL[MxSz][MxSz][MxSz], int cost[MxSz][MxSz], int dist[MxSz], int SubTree[MxSz], int n, int i, int j, int ISF[MxSz][MxSz][MxSz]);

void makeIIF_ISF(int cost[MxSz][MxSz],  int n, int ISF[MxSz][MxSz][MxSz]);

// Runs the Key Link Algorithm modifying condition 2 of Key Links with a path from j to d, instead of u to d
void runISF_steady(int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz]);

//    runISFtemp2: This computes the state right after a FIB update, where it becomes IIF new and if not, ISF temp 2 (runISF temp. with KL variant 2)

void runISF_temp(int orig_cost[MxSz][MxSz], int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz], int ISF_old[MxSz][MxSz][MxSz]);

void runISF_temp1a(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v);

void runISF_temp1b(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v);

void runISF_temp1c(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v);

void runISF_temp2(int orig_cost[MxSz][MxSz], int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz], int ISF_old[MxSz][MxSz][MxSz]);

void runISF_temp_final(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v);
// Traverses the topology from each source to each destination, with each edge failing and the permutation of node states (temp1 vs temp2)
// to check for loops
void traverse_steady_to_temp(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

void traverse_steady_to_temp_to_steady(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

// Traverse the topology in a steady state from each sourch to each destination (checks the table for loops)
void traverse_steady(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

void compare_steady_to_temp(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

void compare_steady_to_temp_to_new(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

int compare_ISF1_to_ISF2(int n, int cost[MxSz][MxSz], int ISF1[MxSz][MxSz][MxSz], int ISF2[MxSz][MxSz][MxSz], int u, int v);

void traverse_temp_to_IIF(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz]);

void traverse_LSA_link_up(int n, int cost[MxSz][MxSz], int weight);


// Raises a base to an exponent
unsigned long long int ipow(unsigned long long int base, int exp);

// A binary counter to evaluate permutations of states
void bin_ctr(int n, int perm[MxSz]);

// From http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y);

/****************************** MAIN ***********************************************/
int main()
{
	int cost[MxSz][MxSz], orig_cost[MxSz][MxSz], new_cost[MxSz][MxSz];
       	int ISF[MxSz][MxSz][MxSz];
        int ISF_steady[MxSz][MxSz][MxSz];
        int ISFtemp1a[MxSz][MxSz][MxSz], ISFtemp1b[MxSz][MxSz][MxSz];
        int i = 0, j = 0, n = 0, edges = 0;// flag = 0;
        int u = 0, v = 0;
        unsigned long long int p = 0;
  //      int nh = 0;//temp =0, inf = 0, x = 0, y = 0, z =0, nh = 0, d = 0;
//        int x, y, z;
        int NextHop[MxSz][MxSz];// orig_NextHop[MxSz][MxSz];
//        int dist[MxSz], nh;
        int edge_subset[MxSz*2];
        struct timeval tv1, tv2, diff;
        gettimeofday(&tv1, NULL);

        scanf("%d",&n);
	getchar();
	reset_1Darray(edge_subset, 0);
	reset_2Darray(cost, infinity);
    reset_2Darray(NextHop, infinity);
    reset_3Darray(ISF, infinity);
    reset_3Darray(ISF_steady, infinity);
    reset_3Darray(ISFtemp1a, infinity);
    reset_3Darray(ISFtemp1b, infinity);
// Reads input until -1 is seen

while(1)
{
	scanf("%d", &i);
	getchar();
	if(i == -1)
		break;
	scanf("%d", &j);
	getchar();
	if(j == -1)
		break;
            edges++;
	scanf("%d", &cost[i][j]);

	cost[j][i] = cost[i][j];
	getchar();
}
memcpy(orig_cost, cost,  MxSz*MxSz*sizeof(int));
print_cost(cost, n);
//runISF(cost, n, ISF);
//print_ISF(ISF, cost, n);


 fprintf(stderr, "\n Number of edges is %d, Subsets to consider is %llu \n", edges, ipow(2,edges));

memcpy(cost, orig_cost,  MxSz*MxSz*sizeof(int));

int e = 0;
//for( p = 0; p < ipow(2, edges); p++) //edges
{
/** This creates every subset of failed edges from the given topology **/
        fprintf(stderr, "\n %llu: ", p);
//        printf(stderr, "\n %llu: ", p);
       e = 0;
  /*      for(i = 0; i < n; i++)
        {
            for(j = i + 1; j < n; j++)
            {
                if(cost[i][j] != infinity)
                   {

                        if(edge_subset[e] == 1)
                        {
                           //    printf("Removed %c to %c\n", i +'A', j +'A');
                             cost[i][j] = infinity;
                             cost[j][i] = infinity;
                        }
                        e++;
                    }
            }
        }
*/
        //memcpy(cost, orig_cost,  MxSz*MxSz*sizeof(int));
        //reset_3Darray(ISF, infinity);
        reset_3Darray(ISF_steady, infinity);
        runISF_orig(cost, n, ISF_steady);
        traverse_steady_to_temp(n, cost, ISF_steady);
       // traverse_temp_to_IIF(n, cost, ISF_steady);
        //memcpy(new_cost, cost, MxSz*MxSz*sizeof(int));
        print_ISF(ISF_steady, cost, n);
        u = 2;
        v = 6;
        new_cost[u][v] = infinity;
        new_cost[v][u] = infinity;
        runISF_temp1a(cost, new_cost, n, ISF_steady, ISFtemp1a, u, v);
                                                runISF_temp1b(cost, new_cost, n, ISF_steady, ISFtemp1a, u, v);
                                                runISF_temp1c(cost, new_cost, n, ISF_steady, ISFtemp1a, u, v);

     //   runISF_temp_final(cost, new_cost, n, ISF_steady, ISFtemp1a, u, v);
        print_ISF(ISFtemp1a, new_cost, n);
      //  traverse_no_fail(n, new_cost, ISFtemp1a);
      //  traverse_temp_to_temp(n, cost, ISF_steady);
            /** Traverses all subsets of ISF steady (old) with the ISF temp (After LSA received and FIB computed) **/
    // compare_steady_to_temp(n, cost, ISF_steady);
    //compare_steady_to_temp_to_new(n, cost, ISF_steady);
    //compare_ISF1_to_ISF2(n, cost, ISF_steady, ISF);
    // traverse_steady_to_temp(n, cost, ISF_steady);
    //traverse_steady_to_temp_to_steady(n, cost, ISF_steady);
     //  traverse_temp_to_steady(n, cost, ISF_steady);
   /*for( u = 0; u < n; u ++)
        {
                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                {
                        memcpy(new_cost, cost, MxSz*MxSz*sizeof(int));
                        if(cost[u][v] == infinity)
                                continue;
                        else
                        {
                                new_cost[u][v] = infinity;
                                new_cost[v][u] = infinity;

                                printf("\n\n%c - %c Fail\n", u + 'A', v + 'A');
                                 reset_3Darray(ISFtemp1a, infinity);
                                runISF_temp_final(cost, new_cost, n, ISF_steady, ISFtemp1a, u, v);

                                //runISF_temp1a(cost, new_cost, n, ISF_steady, ISFtemp1b, u, v);
                                //runISF_temp1b(cost, new_cost, n, ISF_steady, ISFtemp1b, u, v);
                                //runISF_temp1c(cost, new_cost, n, ISF_steady, ISFtemp1b, u, v);
printf("\n\n<<<*** ORIGINAL ISF ***>>>\n\n");
                                //if
                                (compare_ISF1_to_ISF2(n, new_cost, ISF_steady, ISFtemp1a, u, v));
                                {

                                   // print_ISF(ISF_steady, cost, n);
                                    //runISF_orig(new_cost, n, ISF);
                                    makeIIF_ISF(new_cost, n, ISF);
                                   printf("\n\n<<<*** difference in ISF Temp and IIF new ***>>>\n\n");
                                    compare_ISF1_to_ISF2(n, new_cost, ISFtemp1a, ISF,u,v);

                                }
                                //runISF_orig(new_cost, n, ISF);
                                //if(compare_ISF1_to_ISF2(n, new_cost, ISF_steady, ISF))
                                 //   printf("\n\n<<<*** difference in ISF old and ISF new ***>>>\n\n");
                        }
                }
        }*/
         memcpy(cost, orig_cost,  MxSz*MxSz*sizeof(int));
        // BINARY COUNTER
        bin_ctr(edges, edge_subset);
}
gettimeofday(&tv2, NULL);

            timeval_subtract(&diff, &tv2, &tv1); // result of function returns 1 if negative 0 is positive
             fprintf(stderr, "\n Number of edges is %d, Subsets considered %llu time %ld.%ld\n", edges, ipow(2,edges), diff.tv_sec, diff.tv_usec);
            return 0;
}

/****************************** END MAIN ***********************************************/

unsigned long long int ipow(unsigned long long int base, int exp)
{
    unsigned long long int result = 1;
    while (exp)
    {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}

void bin_ctr(int n, int subset[MxSz])
{
       int t = 0;
                //        printf("\n");
                while(subset[t] == 1 && t < n)
                {
                         subset[t++] = 0;
                }
                subset[t] = 1;
               // for(t = 0; t < n; t++)
                //     printf("%d ", subset[t]);
}

// This takes an ISF of the steady topo and starts to test permutations of states of nodes that have updated due to each failure.

void traverse_steady_to_temp(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int s = 0, d = 0, v = 0, nh = 0, inf = 0, temp = 0, u = 0, t = 0, w = 0, lc = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int dist[MxSz], NextHop[MxSz][MxSz], state[MxSz];
        int NextHop_t[MxSz][MxSz], ISF_t[MxSz][MxSz][MxSz], ISF_new[MxSz][MxSz][MxSz];;
        int orig_cost[MxSz][MxSz], origNextHop[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];
//        int flag = 0, x ,y ,z;
        reset_1Darray(state, 0);
        reset_1Darray(loop, 0);
        reset_1Darray(loopOut, 0);
        reset_3Darray(ISF_new, infinity);
        reset_3Darray(ISF_t, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }
        memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for(s = 0; s < n; s++)
        {
                fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                   if( s == d )
                        continue;
                   else
                   {
                        for( u = 0; u < n; u ++)
                        {
                                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                                {
                                        memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        if(cost[u][v] == infinity)
                                                continue;
                                        else
                                        {
                                                cost[u][v] = infinity;
                                                cost[v][u] = infinity;

                                            //    printf("%c - %c Fail\n", u + 'A', v + 'A');
                                                for(t = 0;t < n; t++)
                                                {
                                                        dij(n, t, cost, dist, NextHop_t);
                                                }

                                               // runISF_temp1c(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1a(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1b(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1c(orig_cost, cost, n, ISF, ISF_t, u, v);
                                               // compare_ISF1_to_ISF2(n, cost, ISF, ISF_t);
 //                                               memcpy(ISF_t_orig, ISF_t, MxSz*MxSz*MxSz*sizeof(int));
                                              //  print_ISF(ISF_t, cost, n);

/* for loop of w, runs through every permutation of each node in each state for the failure of u-v
 lc is a loop counter for tracking purposes to output the path a packet took to create any loop found
 bin_ctr is a binary counter, to go through all permutations of nodes in either ISF old or ISF temp
 loopOut stores the path packet takes, only needed for output to follow if loop occured*/

                                                for(w = 0; w < ipow(2, n); w++)
                                                {
                                                        lc = 0;
                                                        bin_ctr(n, state);
                                                 //       printf("From %c to Destination %c\n", s + 'A', d+'A');
                                                        loopOut[lc++] = s;
                                                        reset_1Darray(loop, 0);

                                                        inf = s;

/*         inf is interface (previous node) to be used in ISF matrix;
        This is start state of packet originating at s */
                                                        if(state[s] == 0)
                                                                nh = NextHop[s][d];
                                                        else
                                                                nh = NextHop_t[s][d];
                                                        loop[s] = 0; // changed from 1 to 0 should not make a difference...

                                                        if(nh == infinity)
                                                                {
                                                                  //     printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                   //     print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }
/* Because node may not be in ISF temp (therefore has not computed anything, it may try to use a down link as NH, if that link is
 the down link then we compute a NH using dij for that node to create BWT, this does not change its ISF*/

                                                        if( cost[s][nh] == infinity)
                                                        {
                                                      // printf(" (BWT).. -> 1 ");
                                                               // dij(n, s, cost, dist, NextHop); // Changed 24 JUN
                                                               nh = NextHop_t[s][d];// Changed 24 JUN
                                                                              //  nh = ISF[s][nh][d]; // Changed to, on 24 JUN
                                                       // printf(" (BWT).. -> 2 %c\n", nh + 'A');
                                                        }
                                                        if(nh == infinity)
                                                         {
  // printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');

                                                            break;
                                                          //   print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                        }
                                                        memcpy(NextHop, origNextHop,  MxSz*MxSz*sizeof(int));
                                                        loop[nh] = 0; // changed from 1 to 0 should not make a difference...

/* while loop begins actual traversal until the nh is the destination*/
                                                        while(nh != d)
                                                        {
                                                                //printf(" traversing %c ->", nh + 'A');
/* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                                                loopOut[lc++] = nh;
                                                                temp = nh;
/* check if state is old (0) or if either... removed the check of u == nh or v ==nh (don't understand what this served) else  temp (1) */

                                                                if(state[nh] == 0) // || u == nh || v == nh) // why u == nh?
                                                                        nh = ISF[nh][inf][d];
                                                                else
                                                                        nh = ISF_t[nh][inf][d];
                                                                if(nh == infinity)
                                                                {
                                                          //      printf("\n interface is %c, nh is %c", inf + 'A', nh + 'A');
                                                           //             printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                        //print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                       // exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                if( cost[temp][nh] == infinity)
                                                                {
                                                                //        printf(" (BWT)$$ -> ");
                                                                        dij(n, temp, cost, dist, NextHop);
                                                                       nh = NextHop[temp][d];
                                                                     //  nh = ISF[temp][nh][d]; // Changed to, on 24 JUN
                                                                }
                                                                inf = temp;
                                                                if(loop[nh] > 2)
                                                                {
                                                                        printf("\n\n <<<<<<<<< LOOP >>>>>>>>> \n\n");
                                                                        runISF_orig(orig_cost, n, ISF_new);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                loop[nh]++;
                                                        }
                                                        loopOut[lc] = nh;
                                                        //printf(" %c     \n", d + 'A');
                                                      /*  if (nh != infinity)
                                                            printf("Path taken: %c", loopOut[0] + 'A');
                                                        for(t = 1; t < lc + 1; t++)
                                                                 printf(" -> %c", loopOut[t] + 'A');
                                                        if (nh != infinity)
                                                        printf("\n");*/
         //                   runISF_orig(cost, n, ISF_new);
//print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                }
                                         /*     for( x = 0; x < n; x++)
                                                        for( y = 0; y < n; y++)
                                                                for( z = 0; z < n; z++)
                                                                    if(ISF[x][y][z] != ISF_t[x][y][z] && ISF[x][y][z] != infinity && ISF_t[x][y][z] != infinity)
                                                                        {
                                                                          //  printf("\n<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n Node is %c, Interface is %c Destination is %c", x + 'A', y + 'A', z + 'A');
                                                                            flag = 1;
                                                                        }
                                                    if(flag == 1)
                                                    {
                                                        print_cost(cost, n);
                                                        print_ISF(ISF, orig_cost, n);
                                                        print_ISF(ISF_t, cost, n);
                                                       // print_topo(cost, n);

                                                        printf("\n&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&\n");
                                                        //exit(EXIT_SUCCESS);
                                                    }
                                                    flag = 0;
*/
                                        }
                                }
                        }
                   }
                }
        }
}

void traverse_steady_to_temp_to_steady(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int s = 0, d = 0, v = 0, nh = 0, inf = 0, temp = 0, u = 0, t = 0, w = 0, lc = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int dist[MxSz], NextHop[MxSz][MxSz], state[MxSz];
        int NextHop_t[MxSz][MxSz], ISF_t[MxSz][MxSz][MxSz], ISF_new[MxSz][MxSz][MxSz];;
        int orig_cost[MxSz][MxSz], origNextHop[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];
//        int flag = 0, x ,y ,z;
        reset_1Darray(state, 0);
        reset_1Darray(loop, 0);
        reset_1Darray(loopOut, 0);
        reset_3Darray(ISF_new, infinity);
        reset_3Darray(ISF_t, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }
        memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for(s = 0; s < n; s++)
        {
                fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                   if( s == d )
                        continue;
                   else
                   {
                        for( u = 0; u < n; u ++)
                        {
                                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                                {
                                        memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        if(cost[u][v] == infinity)
                                                continue;
                                        else
                                        {
                                                cost[u][v] = infinity;
                                                cost[v][u] = infinity;

                                                //printf("1 %c - %c Fail\n", u + 'A', v + 'A');
                                                for(t = 0;t < n; t++)
                                                {
                                                        dij(n, t, cost, dist, NextHop_t);
                                                }
                                                runISF_temp1a(orig_cost, cost, n, ISF, ISF_t, u, v);
                                              //  runISF_temp1b(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1c(orig_cost, cost, n, ISF, ISF_t, u, v);
                                               // runISF_orig(cost, n, ISF_new);
                                                makeIIF_ISF(cost, n, ISF_new);
                                               // compare_ISF1_to_ISF2(n, cost, ISF, ISF_t);
 //                                               memcpy(ISF_t_orig, ISF_t, MxSz*MxSz*MxSz*sizeof(int));
                                                //print_ISF(ISF_t, cost, n);
/* for loop of w, runs through every permutation of each node in each state for the failure of u-v
 lc is a loop counter for tracking purposes to output the path a packet took to create any loop found
 bin_ctr is a binary counter, to go through all permutations of nodes in either ISF old or ISF temp
 loopOut stores the path packet takes, only needed for output to follow if loop occured*/

                                                for(w = 0; w < ipow(2, n); w++)
                                                {
                                                        lc = 0;
                                                        bin_ctr(n, state);
                                                        loopOut[lc++] = s;
                                                        reset_1Darray(loop, 0);
                                                        inf = s;

/*         inf is interface (previous node) to be used in ISF matrix;
        This is start state of packet originating at s */
                                                        if(state[s] == 0)
                                                                nh = NextHop[s][d];
                                                        else
                                                                nh = NextHop_t[s][d];
                                                        loop[s] = 0; // changed from 1 to 0 should not make a difference...

                                                        if(nh == infinity)
                                                                {
                                                                  //     printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                   //     print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }
/* Because node may not be in ISF temp (therefore has not computed anything, it may try to use a down link as NH, if that link is
 the down link then we compute a NH using dij for that node to create BWT, this does not change its ISF*/

                                                        if( cost[s][nh] == infinity)
                                                        {
                                                      // printf(" (BWT).. -> 1 ");
                                                               // dij(n, s, cost, dist, NextHop); // Changed 24 JUN
                                                               nh = NextHop_t[s][d];// Changed 24 JUN
                                                                              //  nh = ISF[s][nh][d]; // Changed to, on 24 JUN
                                                       // printf(" (BWT).. -> 2 %c\n", nh + 'A');
                                                        }
                                                        if(nh == infinity)
                                                         {
  // printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');

                                                            break;
                                                          //   print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                        }
                                                        memcpy(NextHop, origNextHop,  MxSz*MxSz*sizeof(int));
                                                        loop[nh] = 0; // changed from 1 to 0 should not make a difference...

/* while loop begins actual traversal until the nh is the destination*/
                                                        while(nh != d)
                                                        {
                                                                //printf(" traversing %c ->", nh + 'A');
/* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                                                loopOut[lc++] = nh;
                                                                temp = nh;
/* check if state is old (0) or if either... removed the check of u == nh or v ==nh (don't understand what this served) else  temp (1) */

                                                                if(state[nh] == 0) // || u == nh || v == nh) // why u == nh?
                                                                        nh = ISF[nh][inf][d];
                                                                else
                                                                        nh = ISF_t[nh][inf][d];
                                                                if(nh == infinity)
                                                                {
                                                          //      printf("\n interface is %c, nh is %c", inf + 'A', nh + 'A');
                                                           //             printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                        //print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                       // exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                if( cost[temp][nh] == infinity)
                                                                {
                                                                //        printf(" (BWT)$$ -> ");
                                                                        dij(n, temp, cost, dist, NextHop);
                                                                       nh = NextHop[temp][d];
                                                                     //  nh = ISF[temp][nh][d]; // Changed to, on 24 JUN
                                                                }
                                                                inf = temp;
                                                                if(loop[nh] > 2)
                                                                {
                                                                        printf("\n\n <<<<<<<<< LOOP >>>>>>>>> \n\n");
                                                                        runISF_orig(orig_cost, n, ISF_new);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                loop[nh]++;
                                                        }
                                                        loopOut[lc] = nh;
                                                        //printf(" %c     \n", d + 'A');
                                                      /*  if (nh != infinity)
                                                        {
                                                            printf("Path taken in stdy to temp: %c", loopOut[0] + 'A');

                                                        }
                                                        for(t = 1; t < lc + 1; t++)
                                                                 printf(" -> %c", loopOut[t] + 'A');
                                                        if (nh != infinity)
                                                        printf("\n");*/
         //                   runISF_orig(cost, n, ISF_new);
//print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                }

                                                for(w = 0; w < ipow(2, n); w++)
                                                {
                                                        lc = 0;
                                                        bin_ctr(n, state);
                                                        //printf("From %c to Destination %c\n", s + 'A', d+'A');
                                                        loopOut[lc++] = s;
                                                        reset_1Darray(loop, 0);

                                                        inf = s;

/*         inf is interface (previous node) to be used in ISF matrix;
        This is start state of packet originating at s */
                                                        nh = NextHop_t[s][d];
                                                        loop[s] = 0; // changed from 1 to 0 should not make a difference...

                                                        if(nh == infinity)
                                                                {
                                                                  //     printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                   //     print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }

                                                        memcpy(NextHop, origNextHop,  MxSz*MxSz*sizeof(int));
                                                        loop[nh] = 0; // changed from 1 to 0 should not make a difference...

/* while loop begins actual traversal until the nh is the destination*/
                                                        while(nh != d)
                                                        {
                                                                //printf(" traversing %c ->", nh + 'A');
/* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                                                loopOut[lc++] = nh;
                                                                temp = nh;
/* check if state is old (0) or if either... removed the check of u == nh or v ==nh (don't understand what this served) else  temp (1) */
                                                                if(nh == u || nh == v)
                                                                {
                                                                    nh = NextHop_t[nh][d];
                                                                }
                                                                else if(state[nh] == 0) // || u == nh || v == nh) // why u == nh?
                                                                        nh = ISF_t[nh][inf][d];
                                                                else
                                                                        nh = ISF_new[nh][inf][d];
                                                                if(nh == infinity)
                                                                {
                                                          //      printf("\n interface is %c, nh is %c", inf + 'A', nh + 'A');
                                                           //             printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                        //print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                       // exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                if( cost[temp][nh] == infinity)
                                                                {
                                                                  // fprintf(stderr, "<<< Should not happen temp is %c nh is %c?\n", temp +'A', nh +'A');

                                                                      break;//  fprintf(stderr, "<<< Should not happen temp is %c nh is %c?\n", temp +'A', nh +'A');
                                                                   //     dij(n, temp, cost, dist, NextHop);
                                                                  //     nh = NextHop_t[temp][d];
                                                                  //     print_cost(cost,n);
                                                                   //    exit(EXIT_FAILURE);
                                                                     //  nh = ISF[temp][nh][d]; // Changed to, on 24 JUN
                                                                }
                                                                inf = temp;
                                                                if(loop[nh] > 2)
                                                                {
                                                                        printf("\n\n <<<<<<<<< LOOP Between TEMP AND NEW >>>>>>>>> \n\n");
                                                                       // runISF_orig(orig_cost, n, ISF_new);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                loop[nh]++;
                                                        }
                                                        loopOut[lc] = nh;
                                                        //printf(" %c     \n", d + 'A');
                                                    /*    if (nh != infinity)
                                                            printf("Path taken in temp to steady: %c", loopOut[0] + 'A');
                                                        for(t = 1; t < lc + 1; t++)
                                                                 printf(" -> %c", loopOut[t] + 'A');
                                                        if (nh != infinity)
                                                        printf("\n");*/
         //                   runISF_orig(cost, n, ISF_new);
//print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                }

                                        }
                                }
                        }
                   }
                }
        }
}


// Compares an inputted ISF with another ISF (temp1a)

void compare_steady_to_temp(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int v = 0, u = 0;// t = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int ISF_t[MxSz][MxSz][MxSz];
        int ISF_new[MxSz][MxSz][MxSz];
        int orig_cost[MxSz][MxSz];
        int flag = 0, x ,y ,z;

        reset_3Darray(ISF_t, infinity);
        reset_3Darray(ISF_new, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));

        print_ISF(ISF, orig_cost, n);

        for( u = 0; u < n; u ++)
        {
                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                {
                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                        if(cost[u][v] == infinity)
                                continue;
                        else
                        {
                                cost[u][v] = infinity;
                                cost[v][u] = infinity;

                                printf("\n\n%c - %c Fail\n", u + 'A', v + 'A');

                                runISF_temp1b(orig_cost, cost, n, ISF, ISF_t, u, v);
                        }
                       for( x = 0; x < n; x++)
                            for( y = 0; y < n; y++)
                                    for( z = 0; z < n; z++)
                                        if(ISF[x][y][z] != ISF_t[x][y][z] && ISF[x][y][z] != infinity && ISF_t[x][y][z] != infinity)
                                            {
                                                if(x != u && x != v)
                                                {
                                                    printf("\n<<<<<  %c->%c to %c Temp ISF is %c; Old ISF is %c\n", y + 'A', x + 'A', z + 'A',ISF_t[x][y][z] +'A', ISF[x][y][z] + 'A');
                                                    flag = 1;
                                                }
                                            }
                                                if (flag == 1)
                                                    print_ISF(ISF_t, cost, n);
                                                flag = 0;
                      /*          runISF_orig(cost, n, ISF_new);
                        for( x = 0; x < n; x++)
                            for( y = 0; y < n; y++)
                                    for( z = 0; z < n; z++)
                                        if(ISF_new[x][y][z] != ISF_t[x][y][z] && ISF_new[x][y][z] != infinity && ISF_t[x][y][z] != infinity)
                                            {
                                                if(x != u && x != v)
                                                {
                                                    printf("\n<<<<<  %c->%c to %c ISF new is %c; ISF temp is %c\n", y + 'A', x + 'A', z + 'A',ISF_new[x][y][z] +'A', ISF_t[x][y][z] + 'A');
                                                    flag = 1;
                                                }
                                            }
                                                if (flag == 1)
                                                    print_ISF(ISF_new, cost, n);
                                                flag = 0;
                                                */
                }
            }
}

void compare_steady_to_temp_to_new(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int v = 0, u = 0;// t = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int ISF_t[MxSz][MxSz][MxSz];
        int ISF_new[MxSz][MxSz][MxSz];
        int orig_cost[MxSz][MxSz];
        int flag = 0, x ,y ,z;

        reset_3Darray(ISF_t, infinity);
        reset_3Darray(ISF_new, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));

        print_ISF(ISF, orig_cost, n);

        for( u = 0; u < n; u ++)
        {
                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                {
                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                        if(cost[u][v] == infinity)
                                continue;
                        else
                        {
                                cost[u][v] = infinity;
                                cost[v][u] = infinity;

                                printf("\n\n%c - %c Fail\n", u + 'A', v + 'A');

                                runISF_temp1a(orig_cost, cost, n, ISF, ISF_t, u, v);
                        }
                       for( x = 0; x < n; x++)
                            for( y = 0; y < n; y++)
                                    for( z = 0; z < n; z++)
                                        if(ISF[x][y][z] != ISF_t[x][y][z] && ISF[x][y][z] != infinity && ISF_t[x][y][z] != infinity)
                                            {
                                                if(x != u && x != v)
                                                {
                                                    printf("\n<<<<<  %c->%c to %c Temp ISF is %c; Old ISF is %c\n", y + 'A', x + 'A', z + 'A',ISF_t[x][y][z] +'A', ISF[x][y][z] + 'A');
                                                    flag = 1;
                                                }
                                            }
                                                if (flag == 1)
                                                 {
                                                  //   printf("\n<<<<<  ISF TEMP is >>>>>\n");;
                                                   //  print_ISF(ISF_t, cost, n);
                                                     runISF_orig(cost, n, ISF_t);
                                                     printf("\n<<<<<  ISF NEW is >>>>>\n");;
                                                     print_ISF(ISF_t, cost, n);
                                                 }
                                                flag = 0;
                      /*          runISF_orig(cost, n, ISF_new);
                        for( x = 0; x < n; x++)
                            for( y = 0; y < n; y++)
                                    for( z = 0; z < n; z++)
                                        if(ISF_new[x][y][z] != ISF_t[x][y][z] && ISF_new[x][y][z] != infinity && ISF_t[x][y][z] != infinity)
                                            {
                                                if(x != u && x != v)
                                                {
                                                    printf("\n<<<<<  %c->%c to %c ISF new is %c; ISF temp is %c\n", y + 'A', x + 'A', z + 'A',ISF_new[x][y][z] +'A', ISF_t[x][y][z] + 'A');
                                                    flag = 1;
                                                }
                                            }
                                                if (flag == 1)
                                                    print_ISF(ISF_new, cost, n);
                                                flag = 0;
                                                */
                }
            }
}

int compare_ISF1_to_ISF2(int n, int cost[MxSz][MxSz], int ISF1[MxSz][MxSz][MxSz], int ISF2[MxSz][MxSz][MxSz], int u, int v)
{
      //  int v = 0, u = 0;// t = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
      //  int ISF_t[MxSz][MxSz][MxSz];
      //  int ISF_new[MxSz][MxSz][MxSz];
      //  int orig_cost[MxSz][MxSz];

    int flag = 0, x ,y ,z;

       for( x = 0; x < n; x++)
            for( y = 0; y < n; y++)
                    for( z = 0; z < n; z++)
                        if(ISF1[x][y][z] != ISF2[x][y][z] && ISF1[x][y][z] != infinity && ISF2[x][y][z] != infinity)
                            {
                                    if(x != u && x != v)
                                    {
                                        printf("\n<<<<<  %c->%c to %c ISF 1 is %c; ISF 2 is %c\n", y + 'A', x + 'A', z + 'A',ISF1[x][y][z] +'A', ISF2[x][y][z] + 'A');
                                        flag = 1;
                                    }
                            }

    if (flag == 1)
     {
        printf("\n<<<<<**********  DIFFERENCE FOUND **********>>>>>\n");
      //   printf("\n<<<<<  ISF TEMP is >>>>>\n");;
       //  print_ISF(ISF_t, cost, n);
        // runISF_orig(cost, n, ISF_t);
         printf("\n<<<<<  ISF 1 is >>>>>\n");
         print_ISF(ISF1, cost, n);
         printf("\n<<<<<  ISF 2 is >>>>>\n");
         print_ISF(ISF2, cost, n);
     }
    return flag;
}

/*
    traverse_steady: Takes the steady state of any ISF, fails a link, and traverses to check for loops without any nodes updating their ISF.
*/
void traverse_steady(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int s = 0, d = 0, nh = 0, inf = 0, temp = 0,  lc = 0, v = 0, u;
        int t = 0;
        int dist[MxSz], NextHop[MxSz][MxSz], NextHop_t[MxSz][MxSz], orig_cost[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];

        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));

        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }


        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for(s = 0; s < n; s++)
        {
                //fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                    lc = 0;
                    reset_1Darray(loop, 0);
                    reset_1Darray(loopOut, 0);
                   if( s == d )
                        continue;
                   else
                   {
                        for( u = 0; u < n; u ++)
                        {
                                for( v = u + 1; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                                {
                                        if (u == v || cost[u][v]  == infinity)
                                            continue;
                                        //memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        //memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        reset_1Darray(loop, 0);
                                        lc = 0;
                                        cost[u][v] = infinity;
                                        cost[v][u] = infinity;
                                        printf("Link %c to %c failed\n", u+'A', v+'A');
                                        reset_2Darray(NextHop_t, infinity);

                                        dij(n, u, cost, dist, NextHop_t);
                                        dij(n, v, cost, dist, NextHop_t);

                                        if(s == u || s == v)
                                            nh = NextHop_t[s][d];
                                        else
                                            nh = NextHop[s][d];

                                        if(nh == infinity)
                                        {
                                           // printf("\n <<<<<<<<<  %c is NOT REACHABLE from %c >>>>>>>>> \n",  d + 'A', s + 'A');
                                           // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                            lc = 0;
                                            break;
                                        }
                                        loop[s] = 1;
                                        loopOut[lc++] = s;
                                        loopOut[lc++] = nh;
                                        inf = s;
                /* while loop begins actual traversal until the nh is the destination*/
                                        while(nh != d)
                                        {
                /* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                            temp = nh;
                                            if(nh == u || nh == v)
                                                nh = NextHop_t[nh][d];
                                            else
                                                nh = ISF[nh][inf][d];
                                            inf = temp;
                                            loopOut[lc++] = nh;
                                            if(nh == infinity)
                                            {
                                               //   printf("\n <<<<<<<<<  %c is NOT REACHABLE from %c >>>>>>>>> \n",  d + 'A', s + 'A');
                                               // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                               // if(NextHop_t[u][d] != infinity && NextHop_t[v][d] != infinity)
                                               //         printf("\n <<<<<<<<<  Wrong >>>>>>>>> \n",  d + 'A', s + 'A');
                                                lc = 0;
                                                break;
                                            }
                                                if(loop[nh] >= 2)
                                                {
                                                         printf("\n\n <<<<<<<<< LOOP >>>>>>>>> %d \n\n", nh);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, loop, ISF, ISF, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        exit(EXIT_FAILURE);
                                                }
                                                    loop[nh]++;
                                        }
                                        /* This Prints out the path it took*/
                                        printf("%c", loopOut[0] + 'A');
                                        for(t = 1; t < lc; t++)
                                                 printf(" -> %c", loopOut[t] + 'A');
                                        printf("\n");
                                }
                        }
                }
        }
}
    memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
}

void traverse_no_fail(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int s = 0, d = 0, nh = 0, inf = 0, temp = 0,  lc = 0, v = 0, u;
        int t = 0;
        int dist[MxSz], NextHop[MxSz][MxSz], NextHop_t[MxSz][MxSz], orig_cost[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];

        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));

        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }


        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for(s = 0; s < n; s++)
        {
                //fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                    lc = 0;
                    reset_1Darray(loop, 0);
                    reset_1Darray(loopOut, 0);
                   if( s == d )
                        continue;
                   else
                   {


                                        //memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        //memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        reset_1Darray(loop, 0);
                                        lc = 0;

                                            //fprintf(stderr, "\n Before s is %d, d is % d\n", s ,d);

                                            nh = NextHop[s][d];



                                        if(nh >= infinity)
                                        {
                                            printf("\n <<<<<<<<<  %c is NOT REACHABLE from %c >>>>>>>>> \n",  d + 'A', s + 'A');
                                           // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                           // fprintf(stderr, "\n Number of edges ow(2,edges)");

                                            lc = 0;
                                            continue;
                                        }
                                        //fprintf(stderr, "\n After %d\n", nh);
                                        loop[s] = 1;
                                        loopOut[lc++] = s;
                                        loopOut[lc++] = nh;
                                        inf = s;
                /* while loop begins actual traversal until the nh is the destination*/
                                        while(nh != d)
                                        {
                /* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                            temp = nh;

                                                nh = ISF[nh][inf][d];
                                            inf = temp;
                                            loopOut[lc++] = nh;
                                            if(nh >= infinity)
                                            {
                                                 printf("\n <<<<<<<<<  %c is NOT REACHABLE from %c >>>>>>>>> \n",  d + 'A', s + 'A');
                                               // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                               // if(NextHop_t[u][d] != infinity && NextHop_t[v][d] != infinity)
                                               //         printf("\n <<<<<<<<<  Wrong >>>>>>>>> \n",  d + 'A', s + 'A');
                                                lc = 0;
                                                continue;
                                            }
                                                if(loop[nh] >= 1)
                                                {
                                                         printf("\n\n <<<<<<<<< LOOP >>>>>>>>> %d \n\n", nh);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, loop, ISF, ISF, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                       // exit(EXIT_FAILURE);
                                                }
                                                    loop[nh]++;
                                        }
                                        /* This Prints out the path it took
                                        printf("%c", loopOut[0] + 'A');
                                        for(t = 1; t < lc; t++)
                                                 printf(" -> %c", loopOut[t] + 'A');
                                        printf("\n");*/


                }
        }
}
    memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
}

void traverse_temp_to_IIF(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int s = 0, d = 0, v = 0, nh = 0, inf = 0, temp = 0, u = 0, t = 0, w = 0, lc = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int dist[MxSz], NextHop[MxSz][MxSz], state[MxSz];
        int NextHop_t[MxSz][MxSz];
        int ISF_t[MxSz][MxSz][MxSz], ISF_new[MxSz][MxSz][MxSz];
        int orig_cost[MxSz][MxSz], origNextHop[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];

        reset_1Darray(state, 0);
        reset_1Darray(loop, 0);
        reset_1Darray(loopOut, 0);
        reset_3Darray(ISF_t, infinity);
        reset_3Darray(ISF_new, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }
        memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for(s = 0; s < n; s++)
        {
                fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                   if( s == d )
                        continue;
                   else
                   {
                        for( u = 0; u < n; u ++)
                        {
                                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                                {
                                        memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        if(cost[u][v] == infinity)
                                                continue;
                                        else
                                        {
                                                cost[u][v] = infinity;
                                                cost[v][u] = infinity;
                                                reset_2Darray(NextHop_t, infinity);
                                                for(t = 0;t < n; t++)
                                                {
                                                        dij(n, t, cost, dist, NextHop_t);
                                                }
                                                //print_Dest_NextHop(NextHop_t, n);
                                                //fprintf(stderr, " HERE 1 \n ");
                                                //runISF_steady(cost, n, ISF_t);
                                               /* runISF_temp1a(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1b(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1c(orig_cost, cost, n, ISF, ISF_t, u, v);*/
                                                 runISF_temp_final(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                 makeIIF_ISF(cost, n, ISF_new);
                                              //  runISF_orig(cost, n, ISF_new);
                                                //fprintf(stderr, " HERE 3 \n ");
                                                //memcpy(ISF_t_orig, ISF_t, MxSz*MxSz*MxSz*sizeof(int));
                                                // Compare NH old and NH new, this makes TEMP 2
                                        /*        for(t = 0;t < n; t++)
                                                    for(w = 0; w < n; w++)
                                                        if(NextHop[t][w] != NextHop_t[t][w] && t != w)
                                                            for(inf = 0; inf < n; inf++)
                                                                if(cost[t][inf] != infinity && cost[inf][t] != infinity )
                                                                    ISF_t[t][inf][w] = NextHop_t[t][w];
*/
                               /*         printf("\nin ISF temp change u %c to v %c, t %c inf %c w %c<<<<<<<<<<<<<<<<<<<<<\tISF TEMP\t>>>>>>>>>>>>>>>>>>>>>\n", u + 'A', v + 'A', t + 'A', inf + 'A', w+ 'A');
                                                                        print_ISF(ISF_t, cost, n);
                                printf("\n in ISF temp change <<<<<<<<<<<<<<<<<<<<<\tISF NEW\t>>>>>>>>>>>>>>>>>>>>>\n");
                                                                        print_ISF(ISF_t_orig, cost, n);
                                printf("\nin ISF temp change<<<<<<<<<<<<<<<<<<<<<\tISF OLD\t>>>>>>>>>>>>>>>>>>>>>\n");
                                                                        print_ISF(ISF, orig_cost, n);*/
/* for loop of w, runs through every permutation of each node in each state for the failure of u-v
 lc is a loop counter for tracking purposes to output the path a packet took to create any loop found
 bin_ctr is a binary counter, to go through all permutations of nodes in either ISF old or ISF temp
 loopOut stores the path packet takes, only needed for output to follow if loop occured*/

                                                for(w = 0; w < ipow(2, n); w++)
                                                {
                                                        lc = 0;
                                                        bin_ctr(n, state);
                                                      //printf("%c ->", s + 'A');
                                                        loopOut[lc++] = s;
                                                        reset_1Darray(loop, 0);

                                                        inf = s;

/*         inf is interface (previous node) to be used in ISF matrix;
        This is start state of packet originating at s */
                                                        nh = NextHop_t[s][d];
                                                        loop[s] = 0; // changed from 1 to 0 should not make a difference...
                                                        if(nh == infinity)
                                                            {
                                                                    //printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                   // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                    lc = 0;
                                                                    break;
                                                            }
/* Because node may not be in ISF temp (therefore has not computed anything, it may try to use a down link as NH, if that link is
 the down link then we compute a NH using dij for that node to create BWT, this does not change its ISF*/
                                                        loop[nh] = 0; // changed from 1 to 0 should not make a difference...

/* while loop begins actual traversal until the nh is the destination*/
                                                        while(nh != d)
                                                        {
                                                              //  printf(" %c ->", nh + 'A');
/* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                                                loopOut[lc++] = nh;
                                                                temp = nh;
/* check if state is old (0) or if either... removed the check of u == nh or v ==nh (don't understand what this served) else  temp (1) */
                                               // fprintf(stderr, " HERE 1 %d %d %d\n ", nh, inf, d);

                                                                if(nh == infinity)
                                                                {
                                                                        printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                       // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }
                                                             //   else if(nh == u || nh == v)
                                                              //          nh = NextHop[nh][d];
                                                                else if(state[nh] == 0) // || u == nh || v == nh) // why u == nh?
                                                                        nh = ISF_t[nh][inf][d];
                                                                else
                                                                        nh = ISF_new[nh][inf][d];//ISF_new[nh][inf][d];
                                               // fprintf(stderr, " HERE 2 \n ");

                                                                inf = temp;
                                                                if(loop[nh] >= 2)
                                                                {
                                                                        printf("\n\n <<<<<<<<< LOOP >>>>>>>>> \n\n");
                                                                        runISF_orig(orig_cost, n, ISF_new);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                loop[nh]++;
                                                        }
                                                       // printf(" %c     \n", d + 'A');
                                                }
                                        }
                                }
                        }
                   }
                }
        }
}

void traverse_temp_to_temp(int n, int cost[MxSz][MxSz], int ISF_t[MxSz][MxSz][MxSz])
{
        int s = 0, d = 0, v = 0, nh = 0, inf = 0, temp = 0, u = 0, t = 0, w = 0, lc = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int dist[MxSz], NextHop[MxSz][MxSz], state[MxSz];
        int NextHop_t[MxSz][MxSz];
        //int ISF_t[MxSz][MxSz][MxSz];
        int ISF_new[MxSz][MxSz][MxSz];
        int orig_cost[MxSz][MxSz], origNextHop[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];

        reset_1Darray(state, 0);
        reset_1Darray(loop, 0);
        reset_1Darray(loopOut, 0);
        //reset_3Darray(ISF_t, infinity);
        reset_3Darray(ISF_new, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }
        memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for(s = 0; s < n; s++)
        {
                fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                   if( s == d )
                        continue;
                   else
                   {
                        for( u = 0; u < n; u ++)
                        {
                                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                                {
                                        memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        if(cost[u][v] == infinity)
                                                continue;
                                        else
                                        {
                                                cost[u][v] = infinity;
                                                cost[v][u] = infinity;
                                                reset_2Darray(NextHop_t, infinity);
                                                for(t = 0;t < n; t++)
                                                {
                                                        dij(n, t, cost, dist, NextHop_t);
                                                }
                                                //print_Dest_NextHop(NextHop_t, n);
                                                //fprintf(stderr, " HERE 1 \n ");
                                                //runISF_steady(cost, n, ISF_t);
                                               /* runISF_temp1a(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1b(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                runISF_temp1c(orig_cost, cost, n, ISF, ISF_t, u, v);*/
                                                // runISF_temp_final(orig_cost, cost, n, ISF, ISF_t, u, v);
                                                 //runISF_temp_final_2(orig_cost, cost, n, ISF_t, ISF_new, u, v);
                                                runISF_orig(cost, n, ISF_new);
                                                //fprintf(stderr, " HERE 3 \n ");
                                                //memcpy(ISF_t_orig, ISF_t, MxSz*MxSz*MxSz*sizeof(int));
                                                // Compare NH old and NH new, this makes TEMP 2
                                        /*        for(t = 0;t < n; t++)
                                                    for(w = 0; w < n; w++)
                                                        if(NextHop[t][w] != NextHop_t[t][w] && t != w)
                                                            for(inf = 0; inf < n; inf++)
                                                                if(cost[t][inf] != infinity && cost[inf][t] != infinity )
                                                                    ISF_t[t][inf][w] = NextHop_t[t][w];
*/
                               /*         printf("\nin ISF temp change u %c to v %c, t %c inf %c w %c<<<<<<<<<<<<<<<<<<<<<\tISF TEMP\t>>>>>>>>>>>>>>>>>>>>>\n", u + 'A', v + 'A', t + 'A', inf + 'A', w+ 'A');
                                                                        print_ISF(ISF_t, cost, n);
                                printf("\n in ISF temp change <<<<<<<<<<<<<<<<<<<<<\tISF NEW\t>>>>>>>>>>>>>>>>>>>>>\n");
                                                                        print_ISF(ISF_t_orig, cost, n);
                                printf("\nin ISF temp change<<<<<<<<<<<<<<<<<<<<<\tISF OLD\t>>>>>>>>>>>>>>>>>>>>>\n");
                                                                        print_ISF(ISF, orig_cost, n);*/
/* for loop of w, runs through every permutation of each node in each state for the failure of u-v
 lc is a loop counter for tracking purposes to output the path a packet took to create any loop found
 bin_ctr is a binary counter, to go through all permutations of nodes in either ISF old or ISF temp
 loopOut stores the path packet takes, only needed for output to follow if loop occured*/

                                                for(w = 0; w < ipow(2, n); w++)
                                                {
                                                        lc = 0;
                                                        bin_ctr(n, state);
                                                      //printf("%c ->", s + 'A');
                                                        loopOut[lc++] = s;
                                                        reset_1Darray(loop, 0);

                                                        inf = s;

/*         inf is interface (previous node) to be used in ISF matrix;
        This is start state of packet originating at s */
                                                        nh = NextHop_t[s][d];
                                                        loop[s] = 0; // changed from 1 to 0 should not make a difference...
                                                        if(nh == infinity)
                                                            {
                                                                    //printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                   // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                    lc = 0;
                                                                    break;
                                                            }
/* Because node may not be in ISF temp (therefore has not computed anything, it may try to use a down link as NH, if that link is
 the down link then we compute a NH using dij for that node to create BWT, this does not change its ISF*/
                                                        loop[nh] = 0; // changed from 1 to 0 should not make a difference...

/* while loop begins actual traversal until the nh is the destination*/
                                                        while(nh != d)
                                                        {
                                                              //  printf(" %c ->", nh + 'A');
/* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/
                                                                loopOut[lc++] = nh;
                                                                temp = nh;
/* check if state is old (0) or if either... removed the check of u == nh or v ==nh (don't understand what this served) else  temp (1) */
                                               // fprintf(stderr, " HERE 1 %d %d %d\n ", nh, inf, d);

                                                                if(nh == infinity)
                                                                {
                                                                        printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                       // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }
                                                                else if(nh == u || nh == v)
                                                                        nh = NextHop[nh][d];
                                                                else if(state[nh] == 0) // || u == nh || v == nh) // why u == nh?
                                                                        nh = ISF_t[nh][inf][d];
                                                                else
                                                                        nh = ISF_new[nh][inf][d];//ISF_new[nh][inf][d];
                                               // fprintf(stderr, " HERE 2 \n ");

                                                                inf = temp;
                                                                if(loop[nh] >= 2)
                                                                {
                                                                        printf("\n\n <<<<<<<<< LOOP >>>>>>>>> \n\n");
                                                                        runISF_orig(orig_cost, n, ISF_new);
                                                                        print_loop(n, loopOut, lc, u,  v, s, d, state, ISF_new, ISF_t, ISF_t, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        exit(EXIT_FAILURE);
                                                                        break;
                                                                }
                                                                loop[nh]++;
                                                        }
                                                       // printf(" %c     \n", d + 'A');
                                                }
                                        }
                                }
                        }
                   }
                }
        }
}


// LSA up rc'vd IIF old with ISF new?
void traverse_LSA_link_up(int n, int cost[MxSz][MxSz], int weight)
{
        int s = 0, d = 0, v = 0, nh = 0, inf = 0, temp = 0, u = 0, t = 0, w = 0, lc = 0;// x = 0, y = 0, z = 0 ; // x y z for test 3 inter BWT
        int dist[MxSz], NextHop[MxSz][MxSz], state[MxSz];
        int NextHop_t[MxSz][MxSz], ISF[MxSz][MxSz][MxSz], ISF_t[MxSz][MxSz][MxSz], ISF_t_orig[MxSz][MxSz][MxSz];;
        int orig_cost[MxSz][MxSz], origNextHop[MxSz][MxSz];
        int loop[MxSz];
        int loopOut[MxSz*3];

        reset_1Darray(state, 0);
        reset_1Darray(loop, 0);
        reset_1Darray(loopOut, 0);
        reset_3Darray(ISF, infinity);
        reset_3Darray(ISF_t, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }
        memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on
        runISF_orig(cost, n, ISF);
        for(s = 0; s < n; s++)
        {
                fprintf(stderr, "%d ", s);
                for(d = 0; d < n; d++)
                {
                   if( s == d )
                        continue;
                   else
                   {
                        for( u = 0; u < n; u ++)
                        {
                                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                                {
                                        if (u == v || cost[u][v]  < infinity)
                                            continue;
                                        //memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                                        //memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                                        {
                                                cost[u][v] = weight;
                                                cost[v][u] = weight;
                                                reset_2Darray(NextHop_t, infinity);
                                                printf("\n Added %c to %c weight %d\n", u +'A', v + 'A', weight);
                                                print_cost(cost, n);
                                                for(t = 0;t < n; t++)
                                                {
                                                        dij(n, t, cost, dist, NextHop_t);
                                                }
                                                //print_Dest_NextHop(NextHop_t, n);
                                                //fprintf(stderr, " HERE 1 \n ");
                                                runISF_steady(cost, n, ISF_t);
                                               // fprintf(stderr, " HERE 3 \n ");
                                                memcpy(ISF_t_orig, ISF_t, MxSz*MxSz*MxSz*sizeof(int));
                                                // Compare NH old and NH new, this makes TEMP 2
                                                /*for(t = 0;t < n; t++)
                                                    for(w = 0; w < n; w++)
                                                        if(NextHop[t][w] != NextHop_t[t][w] && t != w)
                                                            for(inf = 0; inf < n; inf++)
                                                                if(cost[t][inf] != infinity && cost[inf][t] != infinity )
                                                                    ISF_t[t][inf][w] = NextHop_t[t][w];
*/
/* for loop of w, runs through every permutation of each node in each state for the failure of u-v
 lc is a loop counter for tracking purposes to output the path a packet took to create any loop found
 bin_ctr is a binary counter, to go through all permutations of nodes in either ISF old or ISF temp
 loopOut stores the path packet takes, only needed for output to follow if loop occured*/

                                                for(w = 0; w < ipow(2, n); w++)
                                                {
                                                        lc = 0;
                                                        bin_ctr(n, state);
                                                      //printf("%c ->", s + 'A');
                                                        loopOut[lc++] = s;
                                                        reset_1Darray(loop, 0);
                                                        state[u] = state[v] = 1;
                                                        inf = s;

/*         inf is interface (previous node) to be used in ISF matrix;
        This is start state of packet originating at s */
                                                        if(state[s] == 0)
                                                            nh = NextHop[s][d];
                                                        else
                                                            nh = NextHop_t[s][d];
                                                        loop[s] = 0; // changed from 1 to 0 should not make a difference...
                                                        if(nh == infinity)
                                                            {
                                                                    printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                   // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                    lc = 0;
                                                                    break;
                                                            }
/* Because node may not be in ISF temp (therefore has not computed anything, it may try to use a down link as NH, if that link is
 the down link then we compute a NH using dij for that node to create BWT, this does not change its ISF*/
                                                        loop[nh] = 0; // changed from 1 to 0 should not make a difference...
                                                        loopOut[lc++] = nh;
/* while loop begins actual traversal until the nh is the destination*/
                                                        while(nh != d)
                                                        {
                                                              //  printf(" %c ->", nh + 'A');
/* Insert the nh into loopOut for error checking if loop found, then temp = nh for checking when to use BWT*/

                                                                temp = nh;
/* check if state is old (0) or if either... removed the check of u == nh or v ==nh (don't understand what this served) else  temp (1) */
                                               // fprintf(stderr, " HERE 1 %d %d %d\n ", nh, inf, d);

                                                                if(nh == infinity)
                                                                {
                                                                        printf("\n <<<<<<<<< Without %c - %c, %c is NOT REACHABLE from %c >>>>>>>>> \n", u + 'A', v + 'A', d + 'A', s + 'A');
                                                                       // print_loop(n, loopOut, lc, u,  v, s, d, state, ISF, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }
                                                                else if(state[nh] == 0) // || u == nh || v == nh) // why u == nh?
                                                                        nh = ISF[nh][inf][d];
                                                                else
                                                                        nh = ISF_t[nh][inf][d];
                                               // fprintf(stderr, " HERE 2 \n ");
                                                                    loopOut[lc++] = nh;
                                                                inf = temp;
                                                                if(loop[nh] >= 3)
                                                                {
                                                                        printf("\n\n <<<<<<<<< LOOP >>>>>>>>> \n\n");
                                                                        print_loop(n, loopOut, lc +3, u,  v, s, d, state, ISF_t, ISF_t, ISF_t_orig, NextHop_t, NextHop, cost, orig_cost);
                                                                        lc = 0;
                                                                        break;
                                                                }
                                                                loop[nh]++;
                                                        }
                                                        printf(" start %c to destination %c \n", s + 'A', d + 'A');
                                                        for(t = 0; t < n; t++)
          printf("%d ", state[t]);
                                                       printf("%c", loopOut[0] + 'A');
                        for(t = 1; t < lc; t++)
                                 printf(" -> %c", loopOut[t] + 'A');
                        printf("\n");
                                                }
                                        }
                                }
                        }
                   }
                }
        }
}

/*
    traverse_stdy_t1: Runs all permutations of a given topo and nodes being in ISF steady(old) and ISF old with KL variant condition 2
    This causes a loop in counterAB topo.
*/

void print_link_down_ISF(int n, int cost[MxSz][MxSz], int ISF[MxSz][MxSz][MxSz])
{
        int  v = 0, inf = 0, u = 0, t = 0, w = 0;
        int dist[MxSz], NextHop[MxSz][MxSz];
        int NextHop_t[MxSz][MxSz], ISF_t[MxSz][MxSz][MxSz], ISF_t_orig[MxSz][MxSz][MxSz];;
        int orig_cost[MxSz][MxSz], origNextHop[MxSz][MxSz];

        reset_3Darray(ISF_t, infinity);
        memcpy(orig_cost, cost, MxSz*MxSz*sizeof(int));
        for(v = 0;v < n; v++)
        {
	        dij(n, v, cost, dist, NextHop);
        }
        memcpy(origNextHop, NextHop, MxSz*MxSz*sizeof(int));
        // Run a loop starting from every node, s, traversing to every node, d, using ISF
        // Starts with dij to begin then uses ISF from then on

        for( u = 0; u < n; u ++)
        {
                for( v = u; v < n; v++) // by starting them equal this tests the default ISF with no failures... or it should?
                {
                        memcpy(NextHop, origNextHop, MxSz*MxSz*sizeof(int));
                        memcpy(NextHop_t, origNextHop, MxSz*MxSz*sizeof(int));
                        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
                        if(cost[u][v] == infinity)
                                continue;
                        else
                        {
                                cost[u][v] = infinity;
                                cost[v][u] = infinity;

                                for(t = 0;t < n; t++)
                                {
                                        dij(n, t, cost, dist, NextHop_t);
                                }
                               // fprintf(stderr, " HERE 1 \n ");
                                runISF_orig(cost, n, ISF_t);
                               // fprintf(stderr, " HERE 3 \n ");
                                memcpy(ISF_t_orig, ISF_t, MxSz*MxSz*MxSz*sizeof(int));
                                // Compare NH old and NH new
                                for(t = 0;t < n; t++)
                                    for(w = 0; w < n; w++)
                                        if(NextHop[t][w] != NextHop_t[t][w] && t != w)
                                            for(inf = 0; inf < n; inf++)
                                                if(cost[t][inf] != infinity && cost[inf][t] != infinity )
                                                    ISF_t[t][inf][w] = NextHop_t[t][w];
printf("\nin ISF temp change u %c to v %c, t %c inf %c w %c<<<<<<<<<<<<<<<<<<<<<\tISF TEMP\t>>>>>>>>>>>>>>>>>>>>>\n", u + 'A', v + 'A', t + 'A', inf + 'A', w+ 'A');
                                                                        print_ISF(ISF_t, cost, n);
                                printf("\n in ISF temp change <<<<<<<<<<<<<<<<<<<<<\tISF NEW\t>>>>>>>>>>>>>>>>>>>>>\n");
                                                                        print_ISF(ISF_t_orig, cost, n);
                                printf("\nin ISF temp change<<<<<<<<<<<<<<<<<<<<<\tISF OLD\t>>>>>>>>>>>>>>>>>>>>>\n");
                                                                        print_ISF(ISF, orig_cost, n);
                        }
                    }
                }
}

// ORIGNAL ISF from PAPER

void runISF_orig(int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz])
{
        int dist[MxSz], NextHop[MxSz][MxSz], origNextHop[MxSz][MxSz], SPT[MxSz][MxSz];
	int orig_cost[MxSz][MxSz]; // Store original costs
	int i, j, u, v, k, m, d; //w

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
                // printf("HERE %d, %d\n", u, v);
				                for(k = 0;k < n; k++)
					          {       // printf("runISF function\n\n");
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
                                                                 //      printf("\n Key Link %c - %c for destination %c added to %c->%c\n", u + 'A', v + 'A', d + 'A', j+ 'A', i+ 'A');
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

        /*printf("\nThe Interface from %c->%c\n", j +'A', i + 'A');
        print_kl_v2(KeyLinks, n, SubTree);*/

              // Compute ISF F^d j->i = R^d i (all edges excluding/KeyLinks^d j->i)
        makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF);
        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
        reset_3Darray(KeyLinks, infinity);
}
}
 //       print_ISF(ISF, orig_cost, n);
 //       print_BWT(BWT, orig_cost, n);
}

/*
    runISFtemp: This computes ISF with KL variant 2. The only difference from runISF is in LINE 8 of algorithm j is the root of tree not u
*/
void runISF_steady(int cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz])
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


// We compute Next Hop...
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
                for(v = 0; v < n; v++)
                {
                       makeSPT(i, v, NextHop, SPT);
                }

// LINE 4 and 5 of ALGORITHM KEY LINKS
                if(NextHop[i][j] != j)
                {
                    //printf("HERE   ISF %c->%c to %c is %c\n", j + 'A', i + 'A', d+ 'A', ISF[i][j][d]+ 'A' );
                    makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF);
                 //  printf("\n###############\t\tNo Key links for %c -> %c\t\t################\n", j +'A', i+ 'A' );
                   continue;
                }
                // We will do for j->i, where j for this will be B


// LINE 6 of ALGORITHM KEY LINKS
	                // Flagging vertices of Subtree
	                // Look at all nodes from SPT of i rooted at j for next hop.
                        // They are flagged in Subtree
                         //makeSubTree(i, j, NextHop, n, SubTree);
                         makeSubTree_v2(j, SPT,n,SubTree);
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
					          {       // printf("runISF function\n\n");
                                                           dij(n, k, cost, dist, NextHop); }
                // Create a new SPT T^u->v u, From u go through each node (m) to compute edges that make SPT (recursive)
                                                reset_2Darray(SPT, infinity);
                     //   printf("HERE 2\n");
                                                for(m = 0; m < n; m++)
                                                      makeSPT(j, m, NextHop, SPT);
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
                                                                   //    printf("\n Key Link %c - %c for destination %c added to %c->%c\n", u + 'A', v + 'A', d + 'A', j+ 'A', i+ 'A');
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

       // printf("\nThe Interface from %c->%c\n", j +'A', i + 'A');
       // print_kl_v2(KeyLinks, n, SubTree);

              // Compute ISF F^d j->i = R^d i (all edges excluding/KeyLinks^d j->i)
        makeISF(KeyLinks, cost, dist, SubTree, n, i, j, ISF);
        memcpy(cost, orig_cost, MxSz*MxSz*sizeof(int));
        reset_3Darray(KeyLinks, infinity);
}
}
 //       print_ISF(ISF, orig_cost, n);
 //       print_BWT(BWT, orig_cost, n);
 //   printf("\nDONE RUN ISF TEMP\n");
}

/*
    runISFtemp2: This computes the state right after a FIB update, where it becomes IIF new and if not,
    ISF temp 2 (runISF temp. with KL variant 2)
*/
void runISF_temp(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz], int ISF_old[MxSz][MxSz][MxSz])
{
        int dist[MxSz], NextHop[MxSz][MxSz];

	int i, j, v, d;

        reset_2Darray(NextHop, infinity);

// We compute Next Hop...
for(v = 0;v < n; v++)
{
	dij(n, v, new_cost, dist, NextHop);
}



for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if(i == j)
                        continue;
                if(new_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {

                    if(d == i)
                        continue;

                    if(NextHop[j][d] == i)
                    {
                        ISF[i][j][d] = NextHop[i][d];
                        //printf("i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(NextHop[j][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }
                    else
                    {
                               // if(origNextHop[i][d] == infinity)
                              //      printf("\n<<<<<<<<THIS SHOULD NOT HAPPEN>>>>>>>>>>>>>>\n");
                            ISF[i][j][d] = ISF_old[i][j][d];

                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}
// Check if it was supposed to send in old, and IS supposed to send.
void runISF_temp1a(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v)
{
        int dist[MxSz], NextHop[MxSz][MxSz], old_NextHop[MxSz][MxSz];;

	int i, j, vert, d;
//    int t, fail_flag = 0;
        reset_2Darray(NextHop, infinity);

// We compute Next Hop...
for(vert= 0;vert < n; vert++)
{
	dij(n, vert, new_cost, dist, NextHop);
	dij(n, vert, old_cost, dist, old_NextHop);
}


for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if((i == j) || (i == u && j  == v) || (i == v && j == u))
                        continue;
                if(old_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {
                    if((i == u) || (i == v))
                    {
                        ISF[i][j][d] = NextHop[i][d];

                    }
                    else if(d == i)
                        continue;
                    /*if(new_cost[i][j] == infinity && old_cost[i][j] != infinity) // ADDED to properly compute NH on nodes adjacent to failure
                    {
                      //  printf("%c, %c, %c\n", i+'A',j+'A',d+'A');
                       // ISF[i][j][d] = NextHop[i][d];
                       fail_flag = 1;
                      // break;
                    }
                    else*/
                    //else if(NextHop[j][d] == i && old_NextHop[j][d] == i) was original 1a, now is 1b here in 1a
                    else if (NextHop[j][d] == i && old_NextHop[i][d] != j)
                    {
                        ISF[i][j][d] = NextHop[i][d];
               //         printf("Temp 1a: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(NextHop[i][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }

                    else
                    {
                               // if(origNextHop[i][d] == infinity)
                              //      printf("\n<<<<<<<<THIS SHOULD NOT HAPPEN>>>>>>>>>>>>>>\n");
                            ISF[i][j][d] = ISF_old[i][j][d];

                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}

void runISF_temp1b(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v)
{
        int dist[MxSz], NextHop[MxSz][MxSz], old_NextHop[MxSz][MxSz];;

	int i, j, vert, d;
//    int t, fail_flag = 0;
        reset_2Darray(NextHop, infinity);

// We compute Next Hop...
for(vert= 0;vert < n; vert++)
{
	dij(n, vert, new_cost, dist, NextHop);
	dij(n, vert, old_cost, dist, old_NextHop);
}


for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if((i == j) || (i == u && j  == v) || (i == v && j == u))
                        continue;
                if(old_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {
                    if((i == u) || (i == v))
                    {
                        ISF[i][j][d] = NextHop[i][d];

                    }
                    else if(d == i)
                        continue;
                    /*if(new_cost[i][j] == infinity && old_cost[i][j] != infinity) // ADDED to properly compute NH on nodes adjacent to failure
                    {
                      //  printf("%c, %c, %c\n", i+'A',j+'A',d+'A');
                       // ISF[i][j][d] = NextHop[i][d];
                       fail_flag = 1;
                      // break;
                    }
                    else*/
                    else if(NextHop[j][d] == i && old_NextHop[i][d] != j) // adding a condition to make sure was not originally unusual
                    {
                        ISF[i][j][d] = NextHop[i][d];
                  //      printf("Temp 1b: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(NextHop[i][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }

                    else
                    {
                               // if(origNextHop[i][d] == infinity)
                           ;   //      printf("\n<<<<<<<<THIS SHOULD NOT HAPPEN>>>>>>>>>>>>>>\n");
                      //      ISF[i][j][d] = ISF_old[i][j][d];

                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}

void runISF_temp1c(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v)
{
        int dist[MxSz], NextHop[MxSz][MxSz], old_NextHop[MxSz][MxSz];;

	int i, j, vert, d;
//    int t, fail_flag = 0;
        reset_2Darray(NextHop, infinity);

// We compute Next Hop...
for(vert= 0;vert < n; vert++)
{
	dij(n, vert, new_cost, dist, NextHop);
	dij(n, vert, old_cost, dist, old_NextHop);
}


for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if((i == j) || (i == u && j  == v) || (i == v && j == u))
                        continue;
                if(old_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {
                    if((i == u) || (i == v))
                    {
                        ISF[i][j][d] = NextHop[i][d];

                    }
                    else if(d == i)
                        continue;
                    /*if(new_cost[i][j] == infinity && old_cost[i][j] != infinity) // ADDED to properly compute NH on nodes adjacent to failure
                    {
                      //  printf("%c, %c, %c\n", i+'A',j+'A',d+'A');
                       // ISF[i][j][d] = NextHop[i][d];
                       fail_flag = 1;
                      // break;
                    }
                    else*/
                    else if(ISF_old[i][j][d] == j) // adding a condition to make sure was not originally unusual
                    {
                        ISF[i][j][d] = NextHop[i][d];
                     //   printf("Temp 1c: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }

                    else if(NextHop[i][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }

                    else
                    {
                               // if(origNextHop[i][d] == infinity)
                              //      printf("\n<<<<<<<<THIS SHOULD NOT HAPPEN>>>>>>>>>>>>>>\n");
                           ;// ISF[i][j][d] = ISF_old[i][j][d];

                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}

void runISF_temp_final(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v)
{
        int dist[MxSz], NextHop[MxSz][MxSz], old_NextHop[MxSz][MxSz];;

	int i, j, vert, d;
//    int t, fail_flag = 0;
        reset_2Darray(NextHop, infinity);

// We compute Next Hop...
for(vert= 0;vert < n; vert++)
{
	dij(n, vert, new_cost, dist, NextHop);
	dij(n, vert, old_cost, dist, old_NextHop);
}


for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if((i == j) || (i == u && j  == v) || (i == v && j == u))
                        continue;
                if(old_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {
                    if((i == u) || (i == v))
                    {
                        ISF[i][j][d] = NextHop[i][d];

                    }
                    else if(d == i)
                        continue;
                    /*if(new_cost[i][j] == infinity && old_cost[i][j] != infinity) // ADDED to properly compute NH on nodes adjacent to failure
                    {
                      //  printf("%c, %c, %c\n", i+'A',j+'A',d+'A');
                       // ISF[i][j][d] = NextHop[i][d];
                       fail_flag = 1;
                      // break;
                    }
                    else*/
                    //else if(NextHop[j][d] == i && old_NextHop[j][d] == i) was original 1a, now is 1b here in 1a
                    else if (NextHop[j][d] == i && old_NextHop[i][d] != j)
                    {
                        ISF[i][j][d] = NextHop[i][d];
               //         printf("Temp 1a: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(ISF_old[i][j][d] == j) // adding a condition to make sure was not originally unusual
                    {
                        ISF[i][j][d] = NextHop[i][d];
                     //   printf("Temp 1c: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(NextHop[i][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }

                    else
                    {
                               // if(origNextHop[i][d] == infinity)
                              //      printf("\n<<<<<<<<THIS SHOULD NOT HAPPEN>>>>>>>>>>>>>>\n");
                            ISF[i][j][d] = ISF_old[i][j][d];
                          //  printf("NO Change: %c->%c to %c is %c\n", j + 'A', i + 'A', d + 'A', ISF_old[i][j][d] + 'A');
                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}

void runISF_temp_final_2(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF_old[MxSz][MxSz][MxSz], int ISF[MxSz][MxSz][MxSz], int u, int v)
{
        int dist[MxSz], NextHop[MxSz][MxSz], old_NextHop[MxSz][MxSz];;

	int i, j, vert, d;
//    int t, fail_flag = 0;
        reset_2Darray(NextHop, infinity);

// We compute Next Hop...
for(vert= 0;vert < n; vert++)
{
	dij(n, vert, new_cost, dist, NextHop);
	dij(n, vert, old_cost, dist, old_NextHop);
}


for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if((i == j) || (i == u && j  == v) || (i == v && j == u))
                        continue;
                if(old_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {
                    if(d == i)
                        continue;
                    /*if(new_cost[i][j] == infinity && old_cost[i][j] != infinity) // ADDED to properly compute NH on nodes adjacent to failure
                    {
                      //  printf("%c, %c, %c\n", i+'A',j+'A',d+'A');
                       // ISF[i][j][d] = NextHop[i][d];
                       fail_flag = 1;
                      // break;
                    }
                    else*/
                    //else if(NextHop[j][d] == i && old_NextHop[j][d] == i) was original 1a, now is 1b here in 1a
                    else if (NextHop[j][d] != i && old_NextHop[j][d] == i) // Where j->i != T_new AND j->i = T_old
                    {
                        ISF[i][j][d] = NextHop[i][d];
               //         printf("Temp 1a: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(ISF_old[i][j][d] == j) // adding a condition to make sure was not originally unusual
                    {
                        ISF[i][j][d] = NextHop[i][d];
                     //   printf("Temp 1c: i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                    else if(NextHop[i][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }

                    else
                    {
                               // if(origNextHop[i][d] == infinity)
                              //      printf("\n<<<<<<<<THIS SHOULD NOT HAPPEN>>>>>>>>>>>>>>\n");
                            ISF[i][j][d] = ISF_old[i][j][d];
                          //  printf("NO Change: %c->%c to %c is %c\n", j + 'A', i + 'A', d + 'A', ISF_old[i][j][d] + 'A');
                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}

void runISF_temp2(int old_cost[MxSz][MxSz], int new_cost[MxSz][MxSz], int n, int ISF[MxSz][MxSz][MxSz], int ISF_old[MxSz][MxSz][MxSz])
{
        int dist[MxSz], NextHop[MxSz][MxSz];
        int Old_NextHop[MxSz][MxSz];
       // int ISF_new[MxSz][MxSz][MxSz];
	int i, j, v, d;

        reset_2Darray(NextHop, infinity);
        reset_2Darray(Old_NextHop, infinity);
//        reset_3Darray(ISF_new, infinity);
        //runISF_steady(new_cost, n, ISF_new);
// We compute Next Hop...
for(v = 0;v < n; v++)
{
	dij(n, v, new_cost, dist, NextHop);
}

for(v = 0;v < n; v++)
{
	dij(n, v, old_cost, dist, Old_NextHop);
}

for(i = 0; i < n; i++)
{
        for(j = 0;j < n; j ++)
        {

                if(i == j)
                        continue;
                if(new_cost[i][j] == infinity)
                        continue;

                for(d = 0; d < n; d++)
                {

                    if(d == i)
                        continue;

                    //if(Old_NextHop[i][d] == ISF_old[i][j][d]) THEN   ISF[i][j][d] = NextHop[i][d];
                    if(Old_NextHop[i][d] == j && ISF_old[i][j][d] != j)
                    {
                        ISF[i][j][d] = ISF_old[i][j][d];
                      //  printf("i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }
                   /* else if(NextHop[i][d] == j)
                    {
                        ISF[i][j][d] = ISF_old[i][j][d];
                    }*/
                   /* else if(NextHop[j][d] == i)
                    {
                        ISF[i][j][d] = NextHop[i][d];
                        //printf("i is %c, j is %c, d is %c\n", i + 'A', j + 'A', d + 'A');
                    }*/
                    //else if(Old_NextHop[i][d] ==)
                    else if(NextHop[i][d] == infinity)
                    {
                            //fprintf(stderr, "\n<<<< NOT REACHABLE ISFTEMP2 >>>>\n");
                            ISF[i][j][d] = infinity;
                    }
                    else
                    {

                        ISF[i][j][d] = NextHop[i][d];
                       //     ISF[i][j][d] = ISF_old[i][j][d];

                    }
                }
        }
 //   printf("\nDONE RUN ISF TEMP\n");
}
}

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
        int i = 0;
        for(; i < MxSz; i++)
                array[i] = reset;
}

void print_Dest_NextHop(int NextHop[MxSz][MxSz], int n)
{

	int i = 0, j = 0;
	printf("S is source, D is destination\nS\\D|");
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
				printf(" * |");
			else
				printf(" %c |", NextHop[i][j] + 'A');
		}
		printf("\n");
	}


}

void print_topo(int cost[MxSz][MxSz], int n)
{
    int i, j, e = 0;
    printf("\n%d\n", n);
    for(i = 0; i < n; i ++)
        for( j = i +1; j < n; j++)
        {
            if( cost[i][j] < infinity)
               {
                 printf("%d %d %d\n", i, j, cost[i][j]);
                 e++;
                }
        }
        printf("edges: %d\n", e);
}

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

void makeSPT(int source, int dest, int NextHop[MxSz][MxSz], int SPT[MxSz][MxSz])
{
        if( source == dest)
        ;       // Do nothing
        else if (NextHop[source][dest] == infinity)
                    SPT[source][dest] = 0;
        else if ( NextHop[source][dest] != dest)
                 makeSPT(NextHop[source][dest], dest, NextHop, SPT);
        else
                 SPT[source][dest] = 1;
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

void print_kl_v2(int KL[MxSz][MxSz][MxSz], int n, int SubTree[MxSz])
{

	int i = 0, j = 0, k = 0;
//        int KL2D[MxSz][MxSz];

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

void print_loop(int n, int loopOut[MxSz], int lc, int u, int v, int s, int d, int state[MxSz], int ISF_new[MxSz][MxSz][MxSz], int ISF_temp[MxSz][MxSz][MxSz], int ISF_orig[MxSz][MxSz][MxSz], int NextHop_t[MxSz][MxSz], int NextHop[MxSz][MxSz], int cost[MxSz][MxSz], int orig_cost[MxSz][MxSz])
{
    int t;
    for(t = 0; t < n; t++)
          printf("%d ", state[t]);
    for(t = 0; t < lc; t++)
          printf(" %c ->", loopOut[t] + 'A');
    printf("\n\n <<<<<<<<< LOOP >>>>>>>>> \n\n");
    fprintf(stderr, "\n\n <<<<<<<<< LOOP! >>>>>>>>> \n\n");
    printf("\n<<<<<<<<<<<<<<<<<<<<<\tCOST\t>>>>>>>>>>>>>>>>>>>>>\n");
    print_cost(cost, n);
    printf("\n<<<<<<<<<<<<<<<<<<<<<\tORIGINAL COST\t>>>>>>>>>>>>>>>>>>>>>\n");
    print_cost(orig_cost, n);
    printf("\n Removed %c-%c:\n", u + 'A', v + 'A');
    printf("%c to %c \n", s + 'A', d +'A');
    printf("\n<<<<<<<<<<<<<<<<<<<<<\tNew\t>>>>>>>>>>>>>>>>>>>>>\n");
    print_ISF(ISF_new, cost, n);
    printf("\n<<<<<<<<<<<<<<<<<<<<<\tTEMP\t>>>>>>>>>>>>>>>>>>>>>\n");
    print_ISF(ISF_temp, cost, n);
    printf("\n<<<<<<<<<<<<<<<<<<<<<\tOLD\t>>>>>>>>>>>>>>>>>>>>>\n");
    print_ISF(ISF_orig, orig_cost, n);
    printf("\n\n");
    printf("\n<<<<<<< NH TEMP >>>>>>>\n");
    print_Dest_NextHop(NextHop_t, n);
    printf("\n<<<<<<< NH OLD >>>>>>>\n");
    print_Dest_NextHop(NextHop, n);
    printf("\n\n <<<<<<<<< LOOP End >>>>>>>>> \n\n");

    print_topo(cost, n);

}
void print_ISF(int ISF[MxSz][MxSz][MxSz], int cost[MxSz][MxSz], int n)
{
        int i,j,d;
       // printf("\n<<<<<<<<<<<<<<<<<<<<<\tInterface Specific Forwarding Tables\t>>>>>>>>>>>>>>>>>>>>>\n");
        printf("\n\t\tISF TABLE \n");
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

/*
    Takes an ISF table and using the KL matrix, removes them from the cost matrix to compute the ISF table for all d on j->i
*/
void makeISF(int KL[MxSz][MxSz][MxSz], int cost[MxSz][MxSz], int dist[MxSz], int SubTree[MxSz], int n, int i, int j, int ISF[MxSz][MxSz][MxSz])
{
                // i and j reference j->i interface in which these key links apply
        int u,v, d, d2; // KL is u v d where u and v are the edge which is a keylink to d marked as 1
//        int flag = 0;
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
                //      for(d2 = 0; d2 < n; d2++)
                        { d2 =d;
                        //        if( NH[j][d2] != j)
                                ISF[i][j][d2] = NH[i][d2];
                        }
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
     //   CAN we "Smartly" compute this like the print_kl? or use the fact we just built it ?for(a
}
void makeIIF_ISF(int cost[MxSz][MxSz],  int n, int ISF[MxSz][MxSz][MxSz])
{
                // i and j reference j->i interface in which these key links apply
        int j,i, d; // KL is u v d where u and v are the edge which is a keylink to d marked as 1
//        int flag = 0;
        int dist[MxSz];
        int NH[MxSz][MxSz];
        reset_1Darray(dist, infinity);
        reset_2Darray(NH, infinity);

        for(i = 0; i < n; i++)
        {
            for(j = 0; j < n; j++)
            {
                for(d = 0; d < n; d ++)
                {
                              if(i == j || i == d)
                                continue;
                                //  print_cost(cost,n);
                                dij(n, i, cost, dist, NH);
                              //  printf("\n\n<<<<<<<<<<<<< DEST NH >>>>>>>>>>>>>\n\n");

                                        ISF[i][j][d] = NH[i][d];
                }
	         }
        }
}

/* Subtract the ‘struct timeval’ values X and Y,
   storing the result in RESULT.
   Return 1 if the difference is negative, otherwise 0. */


int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}

void dij(int n,int v,int cost[MxSz][MxSz],int dist[], int NextHop[MxSz][MxSz])
{

 int i,u,count,w,flag[MxSz],min;//NH[MxSz];
 for(i=0;i < n;i++)
 {

	flag[i]=0,dist[i]=cost[v][i];
	//printf("Initial Cost from vertex %c to dest %c is %d\n", v + 'A', i + 'A', dist[i]);
 	//
	if(dist[i] < infinity)
		{
			// Setting Next hop array (initialized as the only connected nodes)
			NextHop[v][i]= i;
		}
	else
		;//NextHop[v][i] = infinity;
 }

 count=2;
 while(count<n)
 {
                                       // printf("HERE in dij, v: %d, u: %d\n", v, u);
  		min=infinity - 1;
			//printf("Vertex in Dij is %c\n", v + 'A');
			//print_cost(cost, n);
  		for(w=0;w<n;w++)
 			  if(dist[w]<min && !flag[w])
  		 		 { min=dist[w],u=w;	}
  if( u < n && u >= 0)
        flag[u]=1;
  else
   {     /*printf("\n Graph is not connected with vertex %c\n", v + 'A'); print_cost(cost, n)*/; return;}
  count++;
  //printf("HERE in dij, v: %d\n", v);
  for(w=0;w<n;w++)
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
		;//printf("NH from vertex %c to dest %c is %c\n", v + 'A', w + 'A', NextHop[v][w] +'A');
 }
}
