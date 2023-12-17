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

    static bool ReadData(const char* path,
                         std::vector<double>* col1,
                         std::vector<double>* col2)
    {
        std::vector<std::vector<double>*> data;
        data.push_back(col1);
        data.push_back(col2);
        return ReadData(path, data);
    }

    static bool ReadData(const char* path,
                         std::vector<double>* col1,
                         std::vector<double>* col2,
                         std::vector<double>* col3,
                         std::vector<double>* col4,
                         std::vector<double>* col5)
    {
        std::vector<std::vector<double>*> data;
        data.push_back(col1);
        data.push_back(col2);
        data.push_back(col3);
        data.push_back(col4);
        data.push_back(col5);
        return ReadData(path, data);
    }

    static bool ReadData(const char* path,
                         std::vector<double>* col1,
                         std::vector<double>* col2,
                         std::vector<double>* col3,
                         std::vector<double>* col4,
                         std::vector<double>* col5,
                         std::vector<double>* col6)
    {
        std::vector<std::vector<double>*> data;
        data.push_back(col1);
        data.push_back(col2);
        data.push_back(col3);
        data.push_back(col4);
        data.push_back(col5);
        data.push_back(col6);
        return ReadData(path, data);
    }

    /**
     * @brief Reads data from CSV file
     * Version for 2 columns.
     * @param path file path
     * @param data vector of pointers to vectors of doubles to store columns data
     * @return true on success false on failure
     */
    static bool ReadData(const char* path, std::vector<std::vector<double>*> data);
};

////////////////////////////////////////////////////////////////////////////////

#endif // LIBMCSIM_TESTS_CSVFILEREADER_H_
