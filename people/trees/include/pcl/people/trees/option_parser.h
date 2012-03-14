/* *************************************************
 *
 * Copyright (2010) Cedric Cagniart
 *
 * Author : Cedric Cagniart 
 * ************************************************* */

#ifndef OPTIONPARSER_H_DEFINED
#define OPTIONPARSER_H_DEFINED

#include <map>
#include <vector>
#include <string>
#include <sstream>
#include <cstdio>
#include <stdexcept>
#include <iostream>


inline std::string buildFilename(const std::string &baseName, int id)
{
	char cfilename[1024]; //buffer for the filename
	sprintf(cfilename, baseName.c_str(), id);
	return std::string(cfilename);
}

inline std::string buildFilename(const std::string &baseName, int id1, int id2)
{
	char cfilename[1024]; //buffer for the filename
	sprintf(cfilename, baseName.c_str(), id1, id2);
	return std::string(cfilename);
}

inline std::string buildFilename(const std::string &baseName, int id0, int id1, int id2)
{
	char cfilename[1024]; //buffer for the filename
	sprintf(cfilename, baseName.c_str(), id0, id1, id2);
	return std::string(cfilename);
}


/// current restriction, flags have to be separated from the argument
class OptionParser
{
    public :
    inline OptionParser(int argc, char**argv);

    template <class T>
    inline const T getOption(const char *optName);

    protected :
    std::vector<std::string > mArgV;
};



inline OptionParser::OptionParser(int argc, char**argv)
{
    for(int i=0;i<argc;++i)
        mArgV.push_back(std::string(argv[i]));
}


template <class T>
inline const T OptionParser::getOption(const char *optName)
{
    T r;
    for(unsigned int i=0;i<mArgV.size()-1;++i)
    {
        //size_t pos = mArgV[i].find(optName);
        //if(pos != std::string::npos)
        if(mArgV[i].compare(optName) == 0)
        {
            std::stringstream ss(mArgV[i+1]);
            ss >> r;
            if( ss.fail() ) throw std::runtime_error(std::string("(E) bad parsing of command line arg ")+std::string(optName));
            std::cout<<"(I) : command line arg: "+std::string(optName)+" has value "<<r<<std::endl;
            return r;
        }
    }
    std::cout<<"(W) : could not parse command line arg: "+std::string(optName)
    +" and reverted to a random/default value of "<<r<<std::endl;
	return r;
}



#endif
