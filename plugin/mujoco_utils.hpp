#ifndef MUJOCO_SIM_UTILS_HPP
#define MUJOCO_SIM_UTILS_HPP

#include <cstdarg>
#include <cstdio>
#include <vector>
#include <stdexcept>
#include <type_traits> // Required for std::is_arithmetic_v

// Define color codes
#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define MAGENTA "\033[35m"
#define BLUE "\033[34m"

void print_base(FILE * stream, const char * color, const char * prefix, const char * format, va_list args);
void print_debug(const char * format, ...);
void print_info(const char * format, ...);
void print_confirm(const char * format, ...);
void print_warning(const char * format, ...);
void print_error(const char * format, ...);


template <typename T1, typename T2>
constexpr bool are_both_numerical() {
    return std::is_arithmetic_v<T1> && std::is_arithmetic_v<T2>;
}

/**
 * \brief Copies elements from one vector to another, not resizing the destination vector.
 */
template<typename T, typename U, bool check_types = false>
void copy_arrays_no_resize(std::vector<T> & dest, const std::vector<U> & src)
{
  if constexpr(check_types)
  {
    if (!std::is_arithmetic_v<T> || !std::is_arithmetic_v<U>)
    {
      throw std::runtime_error("copy_arrays_no_resize only supports arithmetic types when check_types is true.");
    }
  }

  size_t count = std::min(dest.size(), src.size());
  std::copy(src.begin(), src.begin() + count, dest.begin());
}

#endif // MUJOCO_SIM_UTILS_HPP
