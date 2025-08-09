#include "mujoco_utils.hpp"

LogLevel log_level_ = INFO;

void set_log_level(LogLevel level)
{
  log_level_ = level;
}

void print_base(FILE* stream, const char* color, LogLevel level, const char* prefix, const char* format, va_list args)
{
  if (level < log_level_) return;

  std::fprintf(stream, "%s%s", color, prefix);
  std::vfprintf(stream, format, args);
  std::fprintf(stream, "%s", RESET);
  std::fflush(stream);
}

void print_debug(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, BLUE, LogLevel::DEBUG, "[DEBG] ", format, args);
  va_end(args);
}

void print_info(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, GREEN, LogLevel::INFO, "[INFO] ", format, args);
  va_end(args);
}

void print_confirm(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, MAGENTA, LogLevel::CONFIRM, "[CNFM] ", format, args);
  va_end(args);
}

void print_warning(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, YELLOW, LogLevel::WARNING, "[WARN] ", format, args);
  va_end(args);
}

void print_error(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stderr, RED, LogLevel::ERROR, "[ERROR] ", format, args);
  va_end(args);
}

