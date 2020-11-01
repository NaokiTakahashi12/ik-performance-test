
#pragma once

#include "Map.hpp"

#include <string>

namespace CSV {
    template <typename T>
    class Parser : public Map<T> {
        public :
            Parser();
            Parser(const Parser &parser);
            Parser(const Map<T> &map);

            std::string to_string();
            void from_string(const std::string &csv_string);

            void save(const std::string &filename);
            void load(const std::string &csv_string_file);

        protected :
            using StringLines = std::vector<std::string>;
            using StringMap = std::vector<StringLines>;

            const unsigned int precision_of_float = 8;

            const char *circled_char = "",
                       *line_feed = "\n",
                       *initialize_delimiter = "",
                       *csv_delimiter = ",",
                       *unknown_number = "0";

        private :
            StringMap create_string_map_from_string(const std::string &csv_string);

    };
}

