#ifndef LIBMCSIM_TESTS_CSVFILEREADER_H_
#define LIBMCSIM_TESTS_CSVFILEREADER_H_

////////////////////////////////////////////////////////////////////////////////

#include <vector>

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Reads data from CSV file
 * @param path file path
 * @param cols vector of pointers to vectors of doubles to store columns data
 * @return true on success false on failure
 */
bool ReadCsvColumnsDataFromFile(const char* path, std::vector<std::vector<double>*> cols);

template <typename T>
void PackColumnsVectors(std::vector<std::vector<T>*>& cols) {}

template <typename T, typename... Args>
void PackColumnsVectors(std::vector<std::vector<T>*>& cols, std::vector<T>* first, Args... args)
{
    cols.push_back(first);
    PackColumnsVectors(cols, args...);
}

/**
 * @brief Reads data from CSV file
 * @param path file path
 * @param first first column data vector
 * @param args subsequent columns data vectors
 * @return true on success false on failure
 */
template <typename T, typename... Args>
bool ReadCsvDataFromFile(const char* path, std::vector<T>* first, Args... args)
{
    std::vector<std::vector<T>*> cols;
    PackColumnsVectors(cols, first, args...);
    return ReadCsvColumnsDataFromFile(path, cols);
}

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_TESTS_CSVFILEREADER_H_
