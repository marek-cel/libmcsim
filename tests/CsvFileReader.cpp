#include <CsvFileReader.h>

#include <fstream>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////

bool CsvFileReader::ReadData(const char* file_path,
                             std::vector<double>* col1)
{
    std::ifstream ifs(file_path, std::ifstream::in);

    if ( ifs.is_open() )
    {
        std::string line;

        while ( getline(ifs, line) )
        {
            std::stringstream ss(line);

            float c1;
            ss >> c1;
            col1->push_back(c1);
        }

        ifs.close();
    }
    else
    {
        return false;
    }

    return true;


}

////////////////////////////////////////////////////////////////////////////////

bool CsvFileReader::ReadData(const char* file_path,
                             std::vector<double>* col1,
                             std::vector<double>* col2)
{
    std::ifstream ifs(file_path, std::ifstream::in);

    if ( ifs.is_open() )
    {
        std::string line;

        while ( getline(ifs, line) )
        {
            std::stringstream ss(line);

            float c1;
            float c2;

            ss >> c1;
            ss >> c2;

            col1->push_back(c1);
            col2->push_back(c2);
        }

        ifs.close();
    }
    else
    {
        return false;
    }

    return true;
}
