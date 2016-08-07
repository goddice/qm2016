#ifndef PLATFORM_H_
#define PLATFORM_H_

#ifdef __GNUC__
#define sprintf_s snprintf
#define sscanf_s sscanf
#include <sys/time.h>

#else
#define _CRT_SECURE_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <stdint.h> // portable: uint64_t   MSVC: __int64 

// MSVC defines this in winsock2.h!?
typedef struct timeval {
	long tv_sec;
	long tv_usec;
} timeval;

int gettimeofday(struct timeval * tp, struct timezone * tzp)
{
	// Note: some broken versions only have 8 trailing zero's, the correct epoch has 9 trailing zero's
	static const uint64_t EPOCH = ((uint64_t)116444736000000000ULL);

	SYSTEMTIME  system_time;
	FILETIME    file_time;
	uint64_t    time;

	GetSystemTime(&system_time);
	SystemTimeToFileTime(&system_time, &file_time);
	time = ((uint64_t)file_time.dwLowDateTime);
	time += ((uint64_t)file_time.dwHighDateTime) << 32;

	tp->tv_sec = (long)((time - EPOCH) / 10000000L);
	tp->tv_usec = (long)(system_time.wMilliseconds * 1000);
	return 0;
}

#ifndef __cplusplus
#include <stdarg.h>
#else
#include <cstdarg>
extern "C"
{
#endif


#define insane_free(ptr) { free(ptr); ptr = 0; }

int vasprintf(char **strp, const char *fmt, va_list ap);
int asprintf(char **strp, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

int vasprintf(char **strp, const char *fmt, va_list ap)
{
  int r = -1, size = _vscprintf(fmt, ap);

  if ((size >= 0) && (size < INT_MAX))
  {
    *strp = (char *)malloc(size+1); //+1 for null
    if (*strp)
    {
      r = vsnprintf(*strp, size+1, fmt, ap);  //+1 for null
      if ((r < 0) || (r > size))
      {
        insane_free(*strp);
        r = -1;
      }
    }
  }
  else { *strp = 0; }

  return(r);
}

int asprintf(char **strp, const char *fmt, ...)
{
  int r;
  va_list ap;
  va_start(ap, fmt);
  r = vasprintf(strp, fmt, ap);
  va_end(ap);
  return(r);
}
#endif

#endif //PLATFORM_H_
