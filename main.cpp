#include "mbed.h"
#include "uLCD_4DGL.h"
 
DigitalOut led1(LED1);    // Configure LED1 pin as output
DigitalOut led2(LED2);
DigitalOut led3(LED3);
 
uLCD_4DGL uLCD(D1, D0, D2);

int main()
{
    uLCD.printf("\nSelection:\n"); //Default Green on black text
    uLCD.printf("\ngesture UI\n");
    uLCD.printf("\ntilt angle detection\n");
    ThisThread::sleep_for(30s);
    return 0;
}