#ifndef LIBMCSIM_TESTS_CSVFILEREADER_H_
#define LIBMCSIM_TESTS_CSVFILEREADER_H_

////////////////////////////////////////////////////////////////////////////////

#include <vector>

////////////////////////////////////////////////////////////////////////////////

/**
 * @brief The CsvFileReader class
 */
class CsvFileReader
{
public:

    /**
     * @brief Reads data from CSV file
     * Version for 2 columns.
     * @param file_path file path
     * @param data vector of vectors of doubles to store data of column 1
     * @return true on success false on failure
     */
    static bool ReadData(const char* file_path, std::vector<std::vector<double>*> data);
};

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_TESTS_CSVFILEREADER_H_
