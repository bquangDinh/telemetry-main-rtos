/*
 * utilities.c
 *
 *  Created on: Mar 16, 2026
 *      Author: buiqu
 */

#include <string.h>

#include "utilities.h"

void trim_newline(char *s)
{
    size_t len = strlen(s);

    while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n'))
    {
        s[len-1] = '\0';
        len--;
    }
}
