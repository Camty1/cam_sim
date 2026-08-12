#include <stdio.h>
#include "utils.h"

int main(int argc, char **argv) {
    Arena* arena = (Arena*) arena_create(64, alignof(int));
    int* ints = (int*)arena_allocate(arena, sizeof(int) * 2);
    ints[0] = 0;
    ints[1] = 1;
    ints[2] = 2;
    ints[3] = 3;

    for (int i = 0; i < 4; i++)
    {
        printf("%d %d\n", i, ints[i]);
    }

    arena_free(arena);
}
