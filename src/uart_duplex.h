////
//// Created by Jun Kim on 4/19/23.
////
//#include <stdio.h>
//
//
//#ifndef UNTITLED_UART_DUPLEX_H
//#define UNTITLED_UART_DUPLEX_H
//
//#endif //UNTITLED_UART_DUPLEX_H
//
//void uart_putchar(char c, FILE *stream);
//char uart_getchar(FILE *stream);
//
//void uart_init(void);
//
///* http://www.ermicro.com/blog/?p=325 */
//
//FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
//FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);