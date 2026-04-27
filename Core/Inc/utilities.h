/*
 * utilities.h
 *
 *  Created on: Mar 16, 2026
 *      Author: buiqu
 */

#ifndef INC_UTILITIES_H_
#define INC_UTILITIES_H_

/**
 * @brief Trims trailing newline characters from a string. This function takes a string as input and removes any trailing newline characters (e.g., '\n', '\r') from the end of the string. This is useful for cleaning up input strings that may have been read from a file, received via UART, or obtained from other sources where newline characters may be present at the end of the string. The function modifies the input string in place and does not return any value.
 * @param s Pointer to the string to be trimmed. The string should be null-terminated and should have enough allocated space to allow for modification.
 */
void trim_newline(char *s);

#endif /* INC_UTILITIES_H_ */
