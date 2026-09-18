#ifndef PC_GVF_TEST_FIXTURE_READER_HPP_
#define PC_GVF_TEST_FIXTURE_READER_HPP_

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace pc_gvf_test {

class Fixture
{
public:
    static Fixture load(const std::string& path)
    {
        std::ifstream stream(path);
        if (!stream) {
            throw std::runtime_error("cannot open fixture: " + path);
        }
        Fixture fixture;
        std::string line;
        while (std::getline(stream, line)) {
            if (line.empty() || line[0] == '#') {
                continue;
            }
            const std::size_t separator = line.find('=');
            if (separator == std::string::npos) {
                throw std::runtime_error("fixture line has no '=': " + line);
            }
            fixture.values_[line.substr(0, separator)] = line.substr(separator + 1);
        }
        return fixture;
    }

    const std::string& value(const std::string& key) const
    {
        const auto found = values_.find(key);
        if (found == values_.end()) {
            throw std::runtime_error("fixture key is missing: " + key);
        }
        return found->second;
    }

    int integer(const std::string& key) const
    {
        return std::stoi(value(key));
    }

    double scalar(const std::string& key) const
    {
        return std::stod(value(key));
    }

    std::vector<std::string> strings(const std::string& key) const
    {
        return split(value(key));
    }

    std::vector<double> numbers(const std::string& key) const
    {
        std::vector<double> result;
        for (const std::string& item : split(value(key))) {
            result.push_back(std::stod(item));
        }
        return result;
    }

private:
    static std::vector<std::string> split(const std::string& value)
    {
        std::vector<std::string> result;
        std::istringstream stream(value);
        std::string item;
        while (std::getline(stream, item, ',')) {
            result.push_back(item);
        }
        return result;
    }

    std::map<std::string, std::string> values_;
};

}  // namespace pc_gvf_test

#endif  // PC_GVF_TEST_FIXTURE_READER_HPP_
