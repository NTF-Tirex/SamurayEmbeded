#include "helpers.h"

const char base[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

unsigned int getInt16FromTwoInt8(int high, int low)
{
    return ((high & 0xff) << 8) | (low & 0xff);
}

void hexStringToIntArray(char string[], int *array, int *lenght)
{
    int len = strlen(string);
    *lenght = len / 2;
    for (int i = 0; i < *lenght; i++)
    {
        array[i] = getIntFromHexChar(string[i * 2]) * 16 + getIntFromHexChar(string[i * 2 + 1]);
    }
}

int getIntFromHexChar(char sym)
{
    for (int i = 0; i < 16; i++)
    {
        if (sym == base[i])
        {
            return i;
        }
    }
    return 0;
}

void getHexFromHDec(int value, char *hex)
{
    hex[0] = base[(value >> 4) & 0xf];
    hex[1] = base[value & 0xf];
}

int lrc8(int *data, unsigned int len)
{
    int lrc = 0;
    for (unsigned int i = 0; i < len; i++)
    {
        lrc = (lrc + data[i]) & 0xff;
    }
    return (((lrc ^ 0xff) + 1) & 0xff);
}



void logParam(char *message, int value, bool lnEnabled)
{
    Serial.print(message);
    Serial.print(value);
    if (lnEnabled)
    {
        Serial.println("");
    }
}

void logIntArray(int *data, int lenght)
{
    for (int i = 0; i < lenght; i++)
    {
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println("");
}

void logCharArray(char *data, int lenght)
{
    for (int i = 0; i < lenght; i++)
    {
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println("");
}
