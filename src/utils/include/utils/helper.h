#pragma once

namespace helper {
    inline std::string getSourceDirectory() {
        std::string file_path(__FILE__);  // __FILE__ is the full path of the source file
        size_t last_dir_sep = file_path.rfind('/');  // For Unix/Linux path
        if (last_dir_sep == std::string::npos) {
            last_dir_sep = file_path.rfind('\\');  // For Windows path
        }
        if (last_dir_sep != std::string::npos) {
            return file_path.substr(0, last_dir_sep);  // Extract directory path
        }
        return "";  // Return empty string if path not found
    }
}
