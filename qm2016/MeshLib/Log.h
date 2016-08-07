#ifndef LOG_H
#define LOG_H

#include <stdarg.h>
#include <time.h>
#include "Platform.h"
#include <sys/stat.h>
#include <errno.h>
#include <assert.h>
#include <string.h>
#include <algorithm>

// #### Set the DEBUG macro ####
#ifndef DEBUG
#   if !defined(NDEBUG)
#       define DEBUG 1
#   else
#       define DEBUG 0
#   endif
#endif
// #############################


// #### DEFINED macro ##########
// DEFINE(NAME) returns 1 if NAME is defined to 1, 0 otherwise.
#define DEFINED(macro) DEFINED_(macro)
#define macrotest_1 ,
#define DEFINED_(value) DEFINED__(macrotest_##value)
#define DEFINED__(comma) DEFINED___(comma 1, 0)
#define DEFINED___(_, v, ...) v
// #############################


// #### Logging macros #########

enum {
    LOG_VERBOSE = 2,
    LOG_DEBUG   = 3,
    LOG_INFO    = 4,
    LOG_WARN    = 5,
    LOG_ERROR   = 6,
};

#ifndef LOG_LEVEL
#   if DEBUG
#       define LOG_LEVEL LOG_DEBUG
#   else
#       define LOG_LEVEL LOG_INFO
#   endif
#endif

#define LOG(level, msg, ...) do { \
    if (level >= LOG_LEVEL) \
        dolog(level, msg, __func__, __FILE__, __LINE__, ##__VA_ARGS__); \
} while(0)

#define LOG_V(msg, ...) LOG(LOG_VERBOSE, msg, ##__VA_ARGS__)
#define LOG_D(msg, ...) LOG(LOG_DEBUG,   msg, ##__VA_ARGS__)
#define LOG_I(msg, ...) LOG(LOG_INFO,    msg, ##__VA_ARGS__)
#define LOG_W(msg, ...) LOG(LOG_WARN,    msg, ##__VA_ARGS__)
#define LOG_E(msg, ...) LOG(LOG_ERROR,   msg, ##__VA_ARGS__)

#define DIE(msg, ...) do { \
        LOG(LOG_ERROR,   msg, ##__VA_ARGS__); exit(-1);\
    } while(0)

// CHECK is similar to an assert, but the condition is tested even in release
// mode.
#if DEBUG
    #define CHECK(c) assert(c)
#else
    #define CHECK(c) do { \
        if (!(c)) { \
            LOG_E("Error %s %s %d", __func__,  __FILE__, __LINE__); \
            exit(-1); \
        } \
    } while (0)
#endif

// I redefine asprintf so that if the function fails, we just crash the
// application.  I don't see how we can recover from an asprintf fails
// anyway.
#define asprintf(...) CHECK(asprintf(__VA_ARGS__) != -1)
#define vasprintf(...) CHECK(vasprintf(__VA_ARGS__) != -1)

static void sys_log(const char *msg)
{
    printf("%s\n", msg);
    fflush(stdout);
}

inline void dolog(int level, const char *msg,
           const char *func, const char *file, int line, ...);

static double get_log_time()
{
    static double origin = 0;
    double time;
    struct timeval tv;
    gettimeofday(&tv, NULL);
    time = tv.tv_sec + (double)tv.tv_usec / (1000 * 1000);
    if (!origin) origin = time;
    return time - origin;
}

inline void dolog(int level, const char *msg,
           const char *func, const char *file, int line, ...)
{
    const bool use_colors = !DEFINED(__APPLE__);
    char *msg_formatted, *full_msg;
    const char *format;
    char time_str[32] = "";
    va_list args;

    va_start(args, line);
    vasprintf(&msg_formatted, msg, args);
    va_end(args);

    if (use_colors && level >= LOG_WARN) {
        format = "\e[33;31m%s%-60s\e[m %s (%s:%d)";
    } else {
        format = "%s%-60s %s (%s:%d)";
    }

    if (DEFINED(LOG_TIME))
        sprintf(time_str, "%f: ", get_log_time());

    file = file + max(0, (int)strlen(file) - 20); // Truncate file path.
    asprintf(&full_msg, format, time_str, msg_formatted, func, file, line);
    sys_log(full_msg);

    free(msg_formatted);
    free(full_msg);
}

#endif /* LOG_H */

