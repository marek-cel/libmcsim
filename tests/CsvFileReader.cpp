#include <CsvFileReader.h>

#include <fstream>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////

bool CsvFileReader::ReadData(const char* path, std::vector<std::vector<double>*> data)
{
    std::ifstream ifs(path, std::ifstream::in);

    if ( ifs.is_open() )
    {
        std::string line;

        while ( getline(ifs, line) )
        {
            std::stringstream ss(line);

            for ( auto col : data )
            {
                float c;
                ss >> c;
                col->push_back(c);
            }
        }

        ifs.close();
    }
    else
    {
        return false;
    }

    return true;
}
