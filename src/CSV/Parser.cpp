
#include <CSV/Parser.hpp>

#include <iomanip>
#include <sstream>
#include <fstream>

namespace CSV {
    template <typename T>
    Parser<T>::Parser() {
    }

    template <typename T>
    Parser<T>::Parser(const Parser &parser) : Parser() {
        if(this != &parser) {
            this->clear();
            this->data_map = parser.data_map;
        }
    }

    template <typename T>
    Parser<T>::Parser(const Map<T> &map) : Parser() {
        this->clear();
        this->data_map = map.data();
    }

    template <typename T>
    std::string Parser<T>::to_string() {
        std::string csv_string;
        std::size_t maximum_column = 0;

        {
            const char *delimiter = initialize_delimiter;

            for(const auto &p : this->data_map) {
                csv_string += std::exchange(delimiter, csv_delimiter);
                csv_string += circled_char + p.first + circled_char;
                maximum_column = std::max(p.second.size(), maximum_column);
            }
        }
        csv_string += line_feed;

        for(std::size_t i = 0; i < maximum_column; i ++) {
            const char *delimiter = initialize_delimiter;
            std::stringstream ss;

            for(const auto &p : this->data_map) {
                if(ss << std::exchange(delimiter, csv_delimiter); p.second.size() > i) {
                    ss << std::setprecision(precision_of_float) << p.second.at(i);
                }
                else {
                    ss << std::setprecision(precision_of_float) << unknown_number;
                }
            }
            csv_string += ss.str() + line_feed;
        }
        return csv_string;
    }

    template <typename T>
        T to_number(const std::string &);

    template <typename T>
    void Parser<T>::from_string(const std::string &csv_string) {
        this->clear();

        std::vector<std::vector<std::string>> raw_string_map;
        raw_string_map = create_string_map_from_string(csv_string);

        for(unsigned int i = 0; i < raw_string_map.front().size(); i ++) {
            std::string column_name{raw_string_map.at(0).at(i)};

            for(unsigned int j = 1; j < raw_string_map.size(); j ++) {
                this->data_map[column_name].push_back(
                        to_number<T>(raw_string_map.at(j).at(i))
                        );
            }
        }
    }

    template <typename T>
    void Parser<T>::save(const std::string &filename) {
        std::ofstream output_file;
        const auto save_string = to_string();

        output_file.open(filename);
        output_file << save_string;
        output_file.close();
    }

    template <typename T>
    void Parser<T>::load(const std::string &csv_string_file) {
        std::ifstream file{csv_string_file};
        std::string line_buffer;
        std::string csv_string;

        while(std::getline(file, line_buffer)) {
            csv_string += line_buffer + *line_feed;
        }
        this->from_string(csv_string);
    }

    template <typename T>
    typename Parser<T>::StringMap Parser<T>::create_string_map_from_string(const std::string &csv_string) {
        StringMap ret_string_map;
        StringLines row_lines;
        std::string row_line;
        std::stringstream row_line_stream{csv_string};

        while(std::getline(row_line_stream, row_line, *line_feed)) {
            std::stringstream row_line_word_stream{row_line};
            std::string word;
            ret_string_map.push_back(StringLines());

            while(std::getline(row_line_word_stream, word, *csv_delimiter)) {
                ret_string_map.back().push_back(word);
            }
        }
        return ret_string_map;
    }

    template <>
    int to_number(const std::string &number_string) {
        return std::stoi(number_string);
    }

    template <>
    float to_number(const std::string &number_string) {
        return std::stoi(number_string);
    }

    template <>
    double to_number(const std::string &number_string) {
        return std::stoi(number_string);
    }

    template class Parser<int>;
    template class Parser<float>;
    template class Parser<double>;
}

