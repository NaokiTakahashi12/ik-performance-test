
#include <CSV/Map.hpp>

namespace CSV {
    template <typename T>
    Map<T>::Map() {
    }

    template <typename T>
    Map<T>::Map(const Map &map) : Map() {
        if(this != &map) {
            this->clear();
            this->data_map = map.data_map;
        }
    }

    template <typename T>
    void Map<T>::clear() noexcept {
        data_map.clear();
    }

    template <typename T>
    std::size_t Map<T>::size() const noexcept {
        return data_map.size();
    }

    template <typename T>
    bool Map<T>::is_registered(const std::string &column_name) {
        return data_map.count(column_name) != 0;
    }

    template <typename T>
    typename Map<T>::ColumnData Map<T>::get_column(const std::string &column_name) {
        return data_map[column_name];
    }

    template <typename T>
    void Map<T>::append(const std::string &column_name, const ColumnData &column_data) {
        data_map[column_name] = column_data;
    }

    template <typename T>
    const typename Map<T>::MapData &Map<T>::data() const {
        return data_map;
    }

    template <typename T>
    Map<T> Map<T>::operator = (const Map &map) {
        if(this != &map) {
            this->clear();
            this->data_map = map.data_map;
        }
        return *this;
    }

    template <typename T>
    bool Map<T>::operator == (const Map &map) const {
        return this->data_map == map.data_map;
    }

    template <typename T>
    bool Map<T>::operator != (const Map &map) const {
        return this->data_map != map.data_map;
    }

    template class Map<int>;
    template class Map<float>;
    template class Map<double>;
}

