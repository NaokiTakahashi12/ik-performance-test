
#include <IKBenchmark/BenchmarkFileIO.hpp>

#include <stdexcept>

namespace IKBenchmark {
    bool BenchmarkFileIO::save_log(const std::string &filename) {
        if(csv_parser.size() != 0) {
            //! @todo Automatic ext
            csv_parser.save(filename + ".csv");
            return true;
        }
        else {
            throw std::runtime_error("Zero size of CSV map");
            return false;
        }
    }

    void BenchmarkFileIO::clear_log() {
        csv_parser.clear();
    }
};

