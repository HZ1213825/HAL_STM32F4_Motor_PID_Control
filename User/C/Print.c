#include "Print.h"
int fputc(int ch, FILE *f)
{

    HAL_UART_Transmit(&Print_UART, (uint8_t *)&ch, 1, 0xFFff);

    return ch;
}

int fgetc(FILE *f)
{
    uint8_t ch = 0;
    HAL_UART_Receive(&Print_UART, &ch, 1, 0xffff);
    return ch;
}
