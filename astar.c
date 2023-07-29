#include <stdio.h>

char grid[256][257];


int main(int argc, char* argv[])
{
    FILE* f = fopen("grid.txt", "r");
    fclose(f);
    return 0;
}