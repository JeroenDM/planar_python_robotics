#include <iostream>
#include <sys/mman.h>
#include <stdio.h>

using namespace std;

int inst[] = {510, 20, 30, 40, 50, 60, 100, 200, 10000 };

#define INTS 3 * 3

int main()
{
    FILE* out = fopen("int_file", "wb");  // Error checks are needed
    char *ptr = (char *) inst;
    fwrite( ptr, sizeof( int ), INTS, out );
    fclose( out);

    FILE* in = fopen("int_file", "rb");    // Error checks are needed
    int* ints = (int*)mmap(0, INTS * sizeof(int),
                    PROT_READ, MAP_FILE | MAP_PRIVATE, fileno(in),0);
    fclose(in);
    for(int i = 0; i < INTS; ++i) {
            std::cout << ints[i] << std::endl;
    }
    munmap(ints, INTS * sizeof(int));
        return 0;
}