#ifndef MUJOCO_SIM_UTILS_HPP
#define MUJOCO_SIM_UTILS_HPP

#include <cstdarg>
#include <cstdio>

// Define color codes
#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define MAGENTA "\033[35m"
#define BLUE "\033[34m"

void print_base(FILE* stream, const char* color, const char* prefix, const char* format, va_list args);
void print_debug(const char* format, ...);
void print_info(const char* format, ...);
void print_confirm(const char* format, ...);
void print_warning(const char* format, ...);
void print_error(const char* format, ...);

#endif  // MUJOCO_SIM_UTILS_HPP
