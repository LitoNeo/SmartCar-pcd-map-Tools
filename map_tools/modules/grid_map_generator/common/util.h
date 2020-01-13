#ifndef MODULE_UTIL_H
#define MODULE_UTIL_H

#include <vector>
#include <string>

std::vector<std::string> split(const std::string &str,
                               const std::string &delimiter);

bool EndWith(const std::string &original,
             const std::string &pattern);

#endif


