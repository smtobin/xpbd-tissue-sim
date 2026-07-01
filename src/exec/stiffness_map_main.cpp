#include "simulation/StiffnessMapSimulation.hpp"
#include "config/simulation/StiffnessMapSimulationConfig.hpp"


int main(int argc, char **argv) 
{
    if (argc > 1)
    {
        std::string config_filename(argv[1]);
        Config::StiffnessMapSimulationConfig config(YAML::LoadFile(config_filename));
        Sim::StiffnessMapSimulation sim(&config);
        return sim.run();
    }
    else
    {
        std::cerr << "No config file specified!" << std::endl;
    }
}