#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MAXEDGES 100
#define MAX_VERTICES 20
#define INFINITY 65535

struct edge
{
    int src; 
    int dst;
    int cost;
};

struct graph
{
    int num_vertices;
    struct edge edges[MAXEDGES];
    int num_edges;
};

struct graph Graph;
struct graph spt;
struct graph rspt;

int getcost(struct graph g, int src, int dst)
{
    for(int i=0; i<g.num_edges; i++)
    {
        if(g.edges[i].src == src && g.edges[i].dst == dst)
            return g.edges[i].cost;
    }
    return INFINITY;
}

void calculate_spt(int src)
{
    printf("Calculating SPT for node %c\n", src+'A');
    spt.num_vertices = Graph.num_vertices;
    
    int dist = INFINITY;
    
    for (int i=0; i<Graph.num_vertices; i++)
    {
        if (i==src)
            continue;
        dist = getcost(Graph, src, i);
        int intermediary = -1;
        for (int j=0; j<Graph.num_vertices; j++)
        {
            int cost_src_j = getcost(Graph, src, j);
            int cost_j_i = getcost(Graph, j, i);
            
            if (dist > cost_src_j + cost_j_i )
            {
                dist = cost_src_j + cost_j_i;                
                intermediary = j;
            }
        }
        
        if (intermediary = -1 && dist != INFINITY)
        {
            spt.edges[spt.num_edges].cost = dist;
            spt.edges[spt.num_edges].src = src;
            spt.edges[spt.num_edges].dst = i;
            
            spt.num_edges++;
        }
        else if(dist == INFINITY)
        {
            printf("NO path from node %c to %c\n", src+'A', i+'A');
        }
        else if(intermediary != -1) 
        {
            spt.edges[spt.num_edges].cost = getcost(Graph, src, intermediary);
            spt.edges[spt.num_edges].src = src;
            spt.edges[spt.num_edges].dst = intermediary;

            spt.num_edges++;
            
            spt.edges[spt.num_edges].cost = getcost(Graph, intermediary, i);
            spt.edges[spt.num_edges].src = intermediary;
            spt.edges[spt.num_edges].dst = i;
            
            rspt.num_edges++;
        }
    }    
}

void calculate_rspt(int dest)
{
    printf("Calculating R_SPT for node %c\n", dest+'A');
    rspt.num_vertices = Graph.num_vertices;
    
    int dist = INFINITY;
    
    for (int i=0; i<Graph.num_vertices; i++)
    {
        if (i==dest)
            continue;
        dist = getcost(Graph, i, dest);
        int intermediary = -1;
        for (int j=0; j<Graph.num_vertices; j++)
        {
            int cost_i_j = getcost(Graph, i, j);
            int cost_j_dest = getcost(Graph, j, dest);
            
            if (dist > cost_i_j + cost_j_dest )
            {
                dist = cost_i_j + cost_j_dest;                
                intermediary = j;
            }
        }
        
        if (intermediary = -1 && dist != INFINITY)
        {
            rspt.edges[rspt.num_edges].cost = dist;
            rspt.edges[rspt.num_edges].src = i;
            rspt.edges[rspt.num_edges].dst = dest;
            
            rspt.num_edges++;
        }
        else if(dist == INFINITY)
        {
            printf("NO path from node %c to %c", dest+'A', i+'A');
        }
        else if(intermediary != -1) 
        {
            rspt.edges[rspt.num_edges].cost = getcost(Graph, i, intermediary);
            rspt.edges[rspt.num_edges].src = i;
            rspt.edges[rspt.num_edges].dst = intermediary;

            rspt.num_edges++;
            
            rspt.edges[rspt.num_edges].cost = getcost(Graph, intermediary, dest);
            rspt.edges[rspt.num_edges].src = intermediary;
            rspt.edges[rspt.num_edges].dst = dest;
            
            rspt.num_edges++;
        }
    }    
}

void print_spt(struct graph g)
{
    printf("Number of Vertices: %d \t Edges: %d \n",g.num_vertices, g.num_edges );
    printf("src: \t dst: \t cost: \n");
    for(int i=0; i<g.num_edges; i++)
    {
        printf("%c\t%c\t%d\n", g.edges[i].src +'A', g.edges[i].dst+'A', g.edges[i].cost);
    }
}

void calculate_spt_rspt(int src)
{
    calculate_rspt(src);
    calculate_spt(src);
}

void print_spt_rspt()
{
    for (int src=0; src<Graph.num_vertices; src++)
    {
        printf("Printing SPT for node %c\n", src+'A');
        print_spt(spt);
        printf("Printing RSPT for node %c\n", src+'A');
        print_spt(rspt);
    }
}

int main(int argc, char *argv[])
{
    int opt = 0;
        
    char *in_fname = NULL;
   
    // Read command line parameters correctly
    while ( (opt = getopt(argc, argv, "f:")) != -1) 
    {
        switch(opt) 
        {
            case 'f':
                in_fname = optarg;
                printf("\nConsidered topology name is%s", in_fname);
                break;
        }
    }
    
    FILE *fp;
    fp = fopen(in_fname, "r");    
    if (fp == NULL)
    {
        printf("Invalid file \n");
        return 0;
    }
    
    printf("Reading topology from file \n");
    int no_of_nodes;
	fscanf(fp, "%d", &no_of_nodes);
	fgetc(fp);
    
    Graph.num_vertices = no_of_nodes;
    Graph.num_edges = 0;

    int src, dst, cost;
	while(1)
	{	
		fscanf(fp, "%d", &src);
		getc(fp);
		if(src == -1)
			break;
		fscanf(fp, "%d", &dst);
		getc(fp);
		if(dst == -1)
			break;

		fscanf(fp, "%d", &cost);

        Graph.edges[Graph.num_edges].src = src;
        Graph.edges[Graph.num_edges].dst = dst;
        Graph.edges[Graph.num_edges].cost = cost;
        Graph.num_edges++;
        
        getc(fp);			
	}
    fclose(fp);
    printf("Reading topology from file is COMPLETED \n");
    
    print_spt(Graph);
    printf("#######################");
    
    for (int src = 0; src < Graph.num_vertices; src++)
    {
        calculate_spt_rspt(src);
        print_spt_rspt();
    }    
}
