#include "util.h"

std::vector<std::string> split(const std::string &str,
                               const std::string &delimiter){
    std::vector<std::string> res;
    std::string::size_type last_index = str.find_first_not_of(delimiter, 0);
    std::string::size_type index = str.find_first_of(delimiter, last_index);

    while (std::string::npos != index || std::string::npos != last_index)
    {
        res.push_back(str.substr(last_index, index - last_index));
        last_index = str.find_first_not_of(delimiter, index);
        index = str.find_first_of(delimiter, last_index);
    }
    return res;
}

bool EndWith(const std::string &original,
             const std::string &pattern) {
  return original.length() >= pattern.length() &&
         original.substr(original.length() - pattern.length()) == pattern;
}

