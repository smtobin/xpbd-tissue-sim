#include "simulation/FixedCubeSimulation.hpp"
#include "config/simulation/FixedCubeSimulationConfig.hpp"


int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        Config::FixedCubeSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::FixedCubeSimulation sim(&config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
}