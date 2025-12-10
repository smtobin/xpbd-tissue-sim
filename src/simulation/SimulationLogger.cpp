#include "simulation/SimulationLogger.hpp"

#include <filesystem>
#include <iostream>
#include <cassert>

namespace Sim
{

SimulationLogger::SimulationLogger(const std::string& output_filename)
{
    // create the output directory if it doesn't exist
    std::filesystem::create_directories(std::filesystem::path(output_filename).parent_path());

    // create the output file
    _output = std::ofstream(output_filename);

    if (!_output.is_open())
    {
        std::cerr << "Error: output file " << output_filename << " could not be created!" << std::endl;
    }
}

void SimulationLogger::addOutput(const std::string& var_name, const Real* var_ptr)
{
    addOutput(var_name, [var_ptr](){
        return *var_ptr;
    });
}

void SimulationLogger::addOutput(const std::string& var_name, std::function<Real()> func)
{
    _logged_variables.emplace_back(var_name, func);
}

void SimulationLogger::startLogging()
{
    assert(!_logging);  // make sure we are not already logging
    _logging = true;

    // print out the header given the variables
    for (const auto& logged_var : _logged_variables)
    {
        _output << logged_var.name << " ";
    }
    _output << "\n";
}

void SimulationLogger::stopLogging()
{
    _logging = false;
    _output.close();
}

void SimulationLogger::logToFile()
{
    // print the current values of the variables
    for (const auto& logged_var : _logged_variables)
    {
        _output << logged_var.func() << " ";
    }
    _output << "\n";
}
    
} // namespace Sim