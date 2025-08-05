#include "mujoco_utils.hpp"

void print_base(FILE* stream, const char* color, const char* prefix, const char* format, va_list args)
{
  std::fprintf(stream, "%s%s", color, prefix);
  std::vfprintf(stream, format, args);
  std::fprintf(stream, "%s", RESET);
  std::fflush(stream);
}

void print_debug(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, BLUE, "[DEBUG] ", format, args);
  va_end(args);
}

void print_info(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, GREEN, "[INFO] ", format, args);
  va_end(args);
}

void print_confirm(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, MAGENTA, "[INFO] ", format, args);
  va_end(args);
}

void print_warning(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stdout, YELLOW, "[WARNING] ", format, args);
  va_end(args);
}

void print_error(const char* format, ...)
{
  va_list args;
  va_start(args, format);
  print_base(stderr, RED, "[ERROR] ", format, args);
  va_end(args);
}