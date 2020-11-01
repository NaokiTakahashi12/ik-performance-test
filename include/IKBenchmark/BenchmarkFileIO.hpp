
#pragma once

#include <string>

#include "../CSV/Parser.hpp"

namespace IKBenchmark {
    class BenchmarkFileIO {
        public :
            bool save_log(const std::string &filename);
            void clear_log();

        protected :
            CSV::Parser<double> csv_parser;
    };
}
