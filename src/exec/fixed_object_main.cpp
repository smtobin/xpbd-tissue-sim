#include "simulation/FixedObjectSimulation.hpp"
#include "config/simulation/FixedObjectSimulationConfig.hpp"


int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        Config::FixedObjectSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::FixedObjectSimulation sim(&config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
}