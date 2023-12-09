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
     * Version for 1 column.
     * @param file_path file path
     * @param col1 vector of doubles to store data
     * @return true on success false on failure
     */
    static bool ReadData( const char* file_path,
                          std::vector<double>* col1 );

    /**
     * @brief Reads data from CSV file
     * Version for 2 columns.
     * @param file_path file path
     * @param col1 vector of doubles to store data of column 1
     * @param col2 vector of doubles to store data of column 2
     * @return true on success false on failure
     */
    static bool ReadData( const char* file_path,
                          std::vector<double>* col1,
                          std::vector<double>* col2 );
};

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_TESTS_CSVFILEREADER_H_
