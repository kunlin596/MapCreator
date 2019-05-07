#include <boost/program_options.hpp>
#include <string>
#include <iostream>

int main(int argc, char** argv) {
    namespace bpo = boost::program_options;
    bpo::options_description desc("Data set converter");

    desc.add_options()
        ("help,h", "produce help message")
        ("input,i", bpo::value<std::string>()->required(), "input image path")
        ("output,o", bpo::value<std::string>()->default_value("."), "out image path")
        ("format,f", bpo::value<std::string>()->default_value("jpg"), "input image format");

    bpo::variables_map opts;
    bpo::store(bpo::parse_command_line(argc, argv, desc), opts);

    bool badArgs = false;

    try {
        bpo::notify(opts);
    } catch(...) {
        badArgs = true;
    }

    if (opts.count("help") || badArgs) {
        std::cout << "Usage: " << argv[0] << std::endl << std::endl;
        std::cout << desc << std::endl;
        return (1);
    }

    const std::string inputPath = opts["input"].as<std::string>();
    const std::string outputPath = opts["output"].as<std::string>();
    const std::string format = opts["format"].as<std::string>();
}
