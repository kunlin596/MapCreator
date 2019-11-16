//
// Created by linkun on 15/09/15.
//

#include "Utility.h"

namespace MapCreator {

std::string GetString(const char* c_str, size_t len) {
    if (c_str) return std::string(c_str, len);
    else return std::string();
}

}  // namespace MapCreator