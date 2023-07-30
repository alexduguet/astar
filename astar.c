#include <stdio.h>
#include <float.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

#define GRID_WIDTH 16
#define GRID_HEIGHT 16

typedef struct Grid
{
    char data[GRID_HEIGHT][GRID_WIDTH];
    int nrows;
    int start_row;
    int start_column;
    int goal_row;
    int goal_column;
} Grid;

void grid_load(Grid* grid, const char * file_name)
{
    FILE* f = fopen(file_name, "r");
    int row;
    for(row = 0; row < GRID_HEIGHT && !feof(f); row++)
    {
        fscanf(f, "%[^\n]\n", grid->data[row]);
        char* pc = strchr(grid->data[row], '+');
        if(pc != NULL)
        {
            grid->start_row = row;
            grid->start_column = pc - grid->data[row];
        }
        pc = strchr(grid->data[row], 'o');
        if(pc != NULL)
        {
            grid->goal_row = row;
            grid->goal_column = pc - grid->data[row];            
        }
    }
    fclose(f);
    grid->nrows = row;    
}

void grid_print(const Grid* grid)
{
    int row;
    for(row = 0; row < grid->nrows; row++)
        printf("%s\n", grid->data[row]);        
}

Grid grid;

bool is_walkable(int row, int column)
{
    if(row < 0 || row >= GRID_HEIGHT || column < 0 || column >= GRID_WIDTH)
        return false;
    else
        return grid.data[row][column] != 'x';
}

typedef struct Node
{
    int row;
    int column;
    float dist_from_start; // minimal distance from start known so far
    float dist_to_goal; // current estimate of distance to goal
    struct Node* came_from;
    int nb_neighbors;
    struct Node* neighbors[8];
} Node;

Node nodes[GRID_HEIGHT][GRID_WIDTH];

void init_nodes()
{
    int row, column;
    for(row = 0; row < GRID_HEIGHT; row++)
        for(column = 0; column < GRID_WIDTH; column++)
        {
            nodes[row][column].row = row;
            nodes[row][column].column = column;
            nodes[row][column].dist_from_start = FLT_MAX;
            nodes[row][column].dist_to_goal = FLT_MAX;
            nodes[row][column].came_from = NULL;
            nodes[row][column].nb_neighbors = 0;            
        }
}

void add_neighbor(Node* node, Node* neighbor)
{
    node->neighbors[node->nb_neighbors] = neighbor;
    node->nb_neighbors++;
}

void add_neighbors(Node* node)
{
    if(node->nb_neighbors > 0) return;
    int row = node->row;
    int column = node->column;
    if(is_walkable(row, column - 1))
        add_neighbor(node, &nodes[row][column - 1]);
    if(is_walkable(row - 1, column))
        add_neighbor(node, &nodes[row - 1][column]);    
    if(is_walkable(row, column + 1))
        add_neighbor(node, &nodes[row][column + 1]);
    if(is_walkable(row + 1, column))
        add_neighbor(node, &nodes[row + 1][column]);
    if(is_walkable(row - 1, column - 1) && is_walkable(row - 1, column) && is_walkable(row, column - 1))
        add_neighbor(node, &nodes[row - 1][column - 1]);
    if(is_walkable(row + 1, column - 1) && is_walkable(row + 1, column) && is_walkable(row, column - 1))
        add_neighbor(node, &nodes[row + 1][column - 1]);
    if(is_walkable(row - 1, column + 1) && is_walkable(row - 1, column) && is_walkable(row, column + 1))
        add_neighbor(node, &nodes[row - 1][column + 1]);
    if(is_walkable(row + 1, column + 1) && is_walkable(row + 1, column) && is_walkable(row, column + 1))
        add_neighbor(node, &nodes[row + 1][column + 1]);
}

float node_distance(const Node* a, const Node* b)
{
    float dx = fabsf((float)(a->row - b->row));
    float dy = fabsf((float)(a->column - b->column));
    return dx < dy ? dy + 0.5 * dx : dx + 0.5 * dy;
}

float node_compare(const Node* a, const Node* b)
{
    return b->dist_to_goal - a->dist_to_goal;
}

typedef struct NodeHeap
{
    Node* data[256];
    int size;
    float (*compare)(const Node*, const Node*);
} NodeHeap;

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

Node* find_path(Node* start, Node* goal, float h(const Node*, const Node*))
{
    init_nodes();

    NodeHeap heap = {0};
    heap.compare = node_compare;
    
    start->came_from = start;
    start->dist_from_start = 0;
    start->dist_to_goal = node_distance(start, goal);
    heap_insert(&heap, start);

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
            float tentative = current->dist_from_start + node_distance(current, neighbor);
            if(tentative < neighbor->dist_from_start)
            {
                neighbor->came_from = current;
                neighbor->dist_from_start = tentative;
                neighbor->dist_to_goal = tentative + h(neighbor, goal); 
                heap_insert(&heap, neighbor);
            }
        }
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    grid_load(&grid, "grid.txt");

    Node* start = &nodes[grid.start_row][grid.start_column];
    Node* goal = &nodes[grid.goal_row][grid.goal_column];
    find_path(start, goal, node_distance);

    Node* current;
    for(current = goal->came_from; current != start; current = current->came_from)
        grid.data[current->row][current->column] = '.';

    grid_print(&grid);

    return 0;
}