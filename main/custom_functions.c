#include "custom_functions.h"

/**
 * @brief Extract gyro, temperature and accel data
 *
 * @param n: number to be printed in binary
 * @return void
 * */
void print_binary(uint8_t *n)
{
    printf("binary: ");
    for (int i = 7; i >= 0; --i)
    {
        uint8_t mask = 1 << i;
        printf("%u", (*n & mask) ? 1 : 0);
    }
    printf("\n");
}

