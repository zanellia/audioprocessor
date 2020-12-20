#include "MemoryLogger.h"
#include "tinyprintf.h"

// #define BUFFER_SIZE 1024*16  //large buffer size here
extern char log_buffer[BUFFER_SIZE];
extern char * log_ptr;
// char log_buffer[BUFFER_SIZE];
// char * log_ptr;

static void _memorylogger_snprintf(const char * const format, ...);


void MemoryLogger_LogString(const char * const str){
  _memorylogger_snprintf("%s", str);
}

void MemoryLogger_LogStringLn(const char * const str){
  _memorylogger_snprintf("%s\n", str);
}

void MemoryLogger_LogFilenameAndLine(const char * const str, int line){
  _memorylogger_snprintf("%s:%d ", str, line);
}


static void _memorylogger_snprintf(const char * const format, ...){

  //don't allow writing past the buffer
  if(log_ptr >= log_buffer + BUFFER_SIZE){
    return;
  }
  va_list args;
  va_start(args, format);

  int bytesWritten = tfp_vsnprintf(log_ptr, (size_t)(BUFFER_SIZE - (log_ptr - log_buffer)), format, args);
  log_ptr += bytesWritten;

  va_end(args);
}


