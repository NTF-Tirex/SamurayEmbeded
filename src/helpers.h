#include <Arduino.h>



unsigned int getInt16FromTwoInt8(int high, int low);
void hexStringToIntArray(char string[], int *array, int *lenght);
int getIntFromHexChar(char sym);
void getHexFromHDec(int value, char *hex);
int lrc8(int *data, unsigned int len);
void logParam(char *message, int value, bool lnEnabled);
void logIntArray(int *data, int lenght);
void logCharArray(char *data, int lenght);