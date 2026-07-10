/* ft8_debug.h — embedded stub for ft8_lib's debug.h.
 * All LOG() calls compile to nothing: newlib-nano stdio must never be
 * pulled into the DSP path (printf float is disabled project-wide). */
#ifndef _DEBUG_H_INCLUDED_
#define _DEBUG_H_INCLUDED_

#define LOG_DEBUG 0
#define LOG_INFO  1
#define LOG_WARN  2
#define LOG_ERROR 3
#define LOG_FATAL 4

#define LOG(level, ...) do { } while (0)

#endif /* _DEBUG_H_INCLUDED_ */
