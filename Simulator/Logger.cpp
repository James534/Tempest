#include "Logger.h"

SimLogger::SimLogger()
{
}

void SimLogger::Log(std::string s){
    std::cout<<s<<std::endl;
}

void SimLogger::Log(irr::core::vector3df v){
    std::cout << std::to_string(v.X)
                 + " " + std::to_string(v.Y)
                 + " " + std::to_string(v.Z) <<std::endl;
}

void SimLogger::Log(float f){
    std::cout << std::to_string(f) << std::endl;
}
