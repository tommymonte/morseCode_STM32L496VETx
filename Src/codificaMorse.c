/*
 * codificaMorse.c
 *      Author: gr15
 */

#include <string.h>
#include <stdio.h>

void encodeMorse(uint8_t character, uint8_t *morseCode) {

    switch (character) {
        case 'A':
            memcpy(morseCode, ".-", 2);
            break;
        case 'B':
            memcpy(morseCode, "-...", 4);
            break;
        case 'C':
            memcpy(morseCode, "-.-.", 4);
            break;
        case 'D':
            memcpy(morseCode, "-..", 3);
            break;
        case 'E':
            memcpy(morseCode, ".", 1);
            break;
        case 'F':
            memcpy(morseCode, "..-.", 4);
            break;
        case 'G':
            memcpy(morseCode, "--.", 3);
            break;
        case 'H':
            memcpy(morseCode, "....", 4);
            break;
        case 'I':
            memcpy(morseCode, "..", 2);
            break;
        case 'J':
            memcpy(morseCode, ".---", 4);
            break;
        case 'K':
            memcpy(morseCode, "-.-", 3);
            break;
        case 'L':
            memcpy(morseCode, ".-..", 4);
            break;
        case 'M':
            memcpy(morseCode, "--", 2);
            break;
        case 'N':
            memcpy(morseCode, "-.", 2);
            break;
        case 'O':
            memcpy(morseCode, "---", 3);
            break;
        case 'P':
            memcpy(morseCode, ".--.", 4);
            break;
        case 'Q':
            memcpy(morseCode, "--.-", 4);
            break;
        case 'R':
            memcpy(morseCode, ".-.", 3);
            break;
        case 'S':
            memcpy(morseCode, "...", 3);
            break;
        case 'T':
            memcpy(morseCode, "-", 1);
            break;
        case 'U':
            memcpy(morseCode, "..-", 3);
            break;
        case 'V':
            memcpy(morseCode, "...-", 4);
            break;
        case 'W':
            memcpy(morseCode, ".--", 3);
            break;
        case 'X':
            memcpy(morseCode, "-..-", 4);
            break;
        case 'Y':
            memcpy(morseCode, "-.--", 4);
            break;
        case 'Z':
            memcpy(morseCode, "--..", 4);
            break;
        case '0':
            memcpy(morseCode, "-----", 5);
            break;
        case '1':
            memcpy(morseCode, ".----", 5);
            break;
        case '2':
            memcpy(morseCode, "..---", 5);
            break;
        case '3':
            memcpy(morseCode, "...--", 5);
            break;
        case '4':
            memcpy(morseCode, "....-", 5);
            break;
        case '5':
            memcpy(morseCode, ".....", 5);
            break;
        case '6':
            memcpy(morseCode, "-....", 5);
            break;
        case '7':
            memcpy(morseCode, "--...", 5);
            break;
        case '8':
            memcpy(morseCode, "---..", 5);
            break;
        case '9':
            memcpy(morseCode, "----.", 5);
            break;
        case ' ':
        	memcpy(morseCode, "*", 1);
        	break;
        default:
            // Carattere non supportato, gestire di conseguenza
            memset(morseCode, 0, 1);  // Azzera il buffer
    }
}
