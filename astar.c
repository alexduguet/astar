#include <stdio.h>
#include <float.h>
#include <stdbool.h>
#include <math.h>

#define GRID_WIDTH 16
#define GRID_HEIGHT 16

char grid[GRID_HEIGHT][GRID_WIDTH];

typedef struct Point
{
    int x;
    int y;
} Point;

typedef struct Node
{
    Point pos;
    float dist_from_start; // minimal distance from start known so far
    float dist_to_goal; // current estimate of distance to goal
    struct Node* came_from;
    int nb_neighbors;
    struct Node* neighbors[8];
} Node;

Node nodes[GRID_HEIGHT][GRID_WIDTH];

typedef struct NodeHeap
{
    Node* data[256];
    int size;
    float (*compare)(Node*, Node*);
} NodeHeap;

bool is_colliding(int x, int y)
{
    return grid[x][y] == 'x';
}

void add_neighbor(Node* node, Node* neighbor)
{
    node->neighbors[node->nb_neighbors] = neighbor;
    node->nb_neighbors++;
}

void add_neighbors(Node* node)
{
    if(node->nb_neighbors > 0) return;
    int x = node->pos.x;
    int y = node->pos.y;
    if(y > 0)
        if(!is_colliding(x, y - 1))
            add_neighbor(node, &nodes[x][y - 1]);
    if(x > 0)
        if(!is_colliding(x - 1, y))
            add_neighbor(node, &nodes[x - 1][y]);    
    if(y < GRID_HEIGHT - 1)
        if(!is_colliding(x, y + 1))
            add_neighbor(node, &nodes[x][y + 1]);
    if(x < GRID_WIDTH - 1)
        if(!is_colliding(x + 1, y))
            add_neighbor(node, &nodes[x + 1][y]);
    if(y > 0 && x > 0)
        if(!is_colliding(x - 1, y - 1) && (!is_colliding(x - 1, y) && !is_colliding(x, y - 1)))
            add_neighbor(node, &nodes[x - 1][y - 1]);
    if(y > 0 && x < GRID_WIDTH - 1)
        if(!is_colliding(x + 1, y - 1) && (!is_colliding(x + 1, y) && !is_colliding(x, y - 1)))
            add_neighbor(node, &nodes[x + 1][y - 1]);
    if(y < GRID_HEIGHT - 1 && x > 0)
        if(!is_colliding(x - 1, y + 1) && (!is_colliding(x - 1, y) && !is_colliding(x, y + 1)))
            add_neighbor(node, &nodes[x - 1][y + 1]);
    if(y < GRID_HEIGHT - 1 && x < GRID_WIDTH - 1)
        if(!is_colliding(x + 1, y + 1) && (!is_colliding(x + 1, y) && !is_colliding(x, y + 1)))
            add_neighbor(node, &nodes[x + 1][y + 1]);
}

void init_nodes()
{
    int x, y;
    for(x = 0; x < GRID_WIDTH; x++)
        for(y = 0; y < GRID_HEIGHT; y++)
        {
            nodes[x][y].pos.x = x;
            nodes[x][y].pos.y = y;
            nodes[x][y].dist_from_start = FLT_MAX;
            nodes[x][y].dist_to_goal = FLT_MAX;
            nodes[x][y].came_from = NULL;
            nodes[x][y].nb_neighbors = 0;            
        }
}

float dist(Point a, Point b)
{
    float dx = fabsf((float)(a.x - b.x));
    float dy = fabsf((float)(a.y - b.y));
    return dx < dy ? dy + 0.5 * dx : dx + 0.5 * dy;
}

float comp(Node* a, Node* b)
{
    return b->dist_to_goal - a->dist_to_goal;
}

void sift_up(NodeHeap* heap, int idx)
{
    if(idx > 0)
    {
        int parent_idx = (idx - 1) / 2;
        if(heap->compare(heap->data[idx], heap->data[parent_idx]) > 0)
        {
            Node* tmp = heap->data[idx];
            heap->data[idx] = heap->data[parent_idx];
            heap->data[parent_idx] = tmp;
            sift_up(heap, parent_idx);
        }
    }
}

void heap_insert(NodeHeap* heap, Node* node)
{
    heap->size++;
    heap->data[heap->size - 1] = node;
    sift_up(heap, heap->size - 1);
}

void sift_down(NodeHeap* heap, int idx)
{
    int left_child_idx = 2 * idx + 1;
    if(left_child_idx < heap->size)
    {
        int max_idx;
        int right_child_idx = 2 * idx + 2;
        if(right_child_idx < heap->size)
        {
            if(heap->compare(heap->data[left_child_idx], heap->data[right_child_idx]) > 0)
                max_idx = left_child_idx;
            else
                max_idx = right_child_idx;
        }
        else
            max_idx = left_child_idx;
        if(heap->compare(heap->data[idx], heap->data[max_idx]) < 0)
        {
            Node* tmp = heap->data[idx];
            heap->data[idx] = heap->data[max_idx];
            heap->data[max_idx] = tmp;
            sift_down(heap, max_idx);           
        }
    }
}

Node* heap_pop(NodeHeap* heap)
{
    Node* head = heap->data[0];
    heap->data[0] = heap->data[heap->size - 1];
    heap->size--;
    sift_down(heap, 0);
    return head;
}

Node* find_path(Point from, Point to, float h(Point, Point))
{
    init_nodes();

    NodeHeap heap = {0};
    heap.compare = comp;
    
    Node* start = &nodes[from.x][from.y];
    start->came_from = start;
    start->dist_from_start = 0;
    start->dist_to_goal = dist(from, to);
    heap_insert(&heap, start);

    Node* goal = &nodes[to.x][to.y];

    while(heap.size > 0)
    {
        Node* current = heap_pop(&heap);
        if(current == goal)
            return current;
        add_neighbors(current);
        int i;
        for(i = 0; i < current->nb_neighbors; i++)
        {
            Node* neighbor = current->neighbors[i];
            float tentative = current->dist_from_start + dist(current->pos, neighbor->pos);
            if(tentative < neighbor->dist_from_start)
            {
                neighbor->came_from = current;
                neighbor->dist_from_start = tentative;
                float old_dist_to_goal = neighbor->dist_to_goal;
                neighbor->dist_to_goal = tentative + h(neighbor->pos, goal->pos); 
                heap_insert(&heap, neighbor);
            }
        }
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    FILE* f = fopen("grid.txt", "r");
    int row;
    for(row = 0; (row < 256) && !feof(f); row++)
        fscanf(f, "%[^\n]\n", grid[row]);
    fclose(f);
    int nrows = row;

    Point from = {5, 3};
    Point to = {1, 5};
    Node* goal = find_path(from, to, dist);

    Node* current = goal->came_from;
    while(current != &nodes[5][3])
    {
        grid[current->pos.x][current->pos.y] = '.';
        current = current->came_from;
    }

    for(row = 0; row < nrows; row++)
        printf("%s\n", grid[row]);    
    return 0;
}