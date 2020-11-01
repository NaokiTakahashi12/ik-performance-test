
#pragma once

#include <vector>
#include <string>
#include <utility>
#include <map>

namespace CSV {
    template <typename T>
    class Map {
        public :
            using ColumnData = std::vector<T>;
            using MapData = std::map<std::string, ColumnData>;

            Map();
            Map(const Map &map);

            void clear() noexcept;
            std::size_t size() const noexcept;

            bool is_registered(const std::string &column_name);

            ColumnData get_column(const std::string &column_name);

            void append(const std::string &column_name, const ColumnData &column_data);

            const MapData &data() const;

            Map operator = (const Map &map);
            bool operator == (const Map &map) const;
            bool operator != (const Map &map) const;

        protected :
            MapData data_map;

    };
}

