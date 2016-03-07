#include "Logger.h"

Logger::Logger()
{
}

void Logger::Log(std::string s){
    std::cout<<s<<std::endl;
}

void Logger::Log(irr::core::vector3df v){
    std::cout << std::to_string(v.X)
                 + " " + std::to_string(v.Y)
                 + " " + std::to_string(v.Z) <<std::endl;
}
