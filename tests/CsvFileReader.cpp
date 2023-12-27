#include <CsvFileReader.h>

#include <fstream>
#include <sstream>

////////////////////////////////////////////////////////////////////////////////

bool ReadCsvColumnsDataFromFile(const char* path, std::vector<std::vector<double>*> cols)
{
    std::ifstream ifs(path, std::ifstream::in);

    if ( ifs.is_open() )
    {
        std::string line;

        while ( getline(ifs, line) )
        {
            std::stringstream ss(line);

            for ( auto col : cols )
            {
                float val;
                ss >> val;
                col->push_back(val);
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
