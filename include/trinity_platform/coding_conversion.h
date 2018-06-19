#ifndef CODING_CONVERSION_H
#define CODING_CONVERSION_H

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h> 
#include <string.h>
#include <stdbool.h>
#include <iconv.h>

namespace trinity_platform{


class CodingConversion
{
public:
static bool unicode_to_utf8 (char *inbuf, size_t *inlen, char *outbuf, size_t *outlen);
static bool utf8_to_unicode (char *inbuf, size_t *inlen, char *outbuf, size_t *outlen);
static int code_convert(char *from_charset, char *to_charset, char *inbuf, size_t inlen,  char *outbuf, size_t outlen);
static int u2g(char *inbuf, size_t inlen, char *outbuf, size_t outlen);
static int g2u(char *inbuf, size_t inlen, char *outbuf, size_t outlen);

private:
};

}




#endif//CODING_CONVERSION_H
