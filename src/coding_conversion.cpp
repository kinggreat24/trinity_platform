#include "trinity_platform/coding_conversion.h"

using namespace trinity_platform;

bool CodingConversion::unicode_to_utf8 (char *inbuf, size_t *inlen, char *outbuf, size_t *outlen)
{
  /* 目的编码, TRANSLIT：遇到无法转换的字符就找相近字符替换
   *           IGNORE ：遇到无法转换字符跳过*/
  char *encTo = "UTF-8";

/* 源编码 */
  char *encFrom = "UNICODE";

  /* 获得转换句柄
   *@param encTo 目标编码方式
   *@param encFrom 源编码方式
   *
   * */
  iconv_t cd = iconv_open (encTo, encFrom);
  if (cd == (iconv_t)-1)
  {
     perror ("iconv_open");
  }

  /* 需要转换的字符串 */
  printf("inbuf=%s\n", inbuf);

  /* 打印需要转换的字符串的长度 */
  printf("inlen=%d\n", *inlen);


  /* 由于iconv()函数会修改指针，所以要保存源指针 */
  char *tmpin = inbuf;
  char *tmpout = outbuf;
  size_t insize = *inlen;
  size_t outsize = *outlen;

  /* 进行转换
   *@param cd iconv_open()产生的句柄
   *@param srcstart 需要转换的字符串
   *@param inlen 存放还有多少字符没有转换
   *@param tempoutbuf 存放转换后的字符串
   *@param outlen 存放转换后,tempoutbuf剩余的空间
   *
   * */
  size_t ret = iconv (cd, &tmpin, inlen, &tmpout, outlen);
  if (ret == -1)
  {
     perror ("iconv");
  }

  /* 存放转换后的字符串 */
  printf("outbuf=%s\n", outbuf);

  //存放转换后outbuf剩余的空间
  printf("outlen=%d\n", static_cast<int>(*outlen));

  int i = 0;

  for (i=0; i<(outsize- (*outlen)); i++)
  {
     printf("%x\n", outbuf[i]);
  }

  /* 关闭句柄 */
  iconv_close (cd);

  return 0;
}

bool CodingConversion::utf8_to_unicode (char *inbuf, size_t *inlen, char *outbuf, size_t *outlen)
{

  /* 目的编码, TRANSLIT：遇到无法转换的字符就找相近字符替换
   *           IGNORE ：遇到无法转换字符跳过*/
  char *encTo = "UNICODE//IGNORE";
  /* 源编码 */
  char *encFrom = "UTF-8";

  /* 获得转换句柄
   *@param encTo 目标编码方式
   *@param encFrom 源编码方式
   *
   * */
  iconv_t cd = iconv_open (encTo, encFrom);
  if (cd == (iconv_t)-1)
  {
      perror ("iconv_open");
  }

  /* 需要转换的字符串 */
  printf("inbuf=%s\n", inbuf);

  /* 打印需要转换的字符串的长度 */
  printf("inlen=%d\n", *inlen);

  /* 由于iconv()函数会修改指针，所以要保存源指针 */
  char *tmpin = inbuf;
  char *tmpout = outbuf;
  size_t insize = *inlen;
  size_t outsize = *outlen;

  /* 进行转换
   *@param cd iconv_open()产生的句柄
   *@param srcstart 需要转换的字符串
   *@param inlen 存放还有多少字符没有转换
   *@param tempoutbuf 存放转换后的字符串
   *@param outlen 存放转换后,tempoutbuf剩余的空间
   *
   * */
  size_t ret = iconv (cd, &tmpin, inlen, &tmpout, outlen);
  if (ret == -1)
  {
     perror ("iconv");
  }

  /* 存放转换后的字符串 */
  printf("outbuf=%s\n", outbuf);

  //存放转换后outbuf剩余的空间
  printf("outlen=%d\n", *outlen);

  int i = 0;

  for (i=0; i<(outsize- (*outlen)); i++)
  {
     //printf("%2c", outbuf[i]);
     printf("%x\n", outbuf[i]);
  }

  /* 关闭句柄 */
  iconv_close (cd);

  return 0;
}


int CodingConversion::code_convert(char *from_charset, char *to_charset, char *inbuf, size_t inlen,  char *outbuf, size_t outlen)
{  
        iconv_t cd;  
        char **pin = &inbuf;  
        char **pout = &outbuf;  
      
        cd = iconv_open(to_charset, from_charset);  
        if (cd == 0)  
            return -1;  
        memset(outbuf, 0, outlen);  
        if (iconv(cd, pin, &inlen, pout, &outlen) == -1)  
            return -1;  
        iconv_close(cd);  
        *pout = '\0';  
      
        return 0;  
}  
      
int CodingConversion::u2g(char *inbuf, size_t inlen, char *outbuf, size_t outlen)
{  
        return code_convert("utf-8", "gb2312", inbuf, inlen, outbuf, outlen);  
}  
      
int CodingConversion::g2u(char *inbuf, size_t inlen, char *outbuf, size_t outlen) 
{
        return code_convert("gb2312", "utf-8", inbuf, inlen, outbuf, outlen);  
} 

