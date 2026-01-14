#ifndef __ELASTIC_MATERIAL_HPP
#define __ELASTIC_MATERIAL_HPP

#include <string>

#include "config/simobject/ElasticMaterialConfig.hpp"

/** A class for representing an elastic material.
 * Basically just stores material properties
 */
class ElasticMaterial
{
    public:
    using ConfigType = Config::ElasticMaterialConfig;
    
    public:

    explicit ElasticMaterial(const ConfigType* config)
        : _name(config->name()),
          _density(config->density()),
          _E(config->E()),
          _nu(config->nu()),
          _mu_s(config->muS()),
          _mu_k(config->muK()),
          _c(config->specificHeat()),
          _k(config->thermalConductivity()),
          _sigma(config->electricalConductivity())
    {
        // calculate Lame parameters
        _mu = _E / (2 * (1 + _nu));
        _lambda = (_E*_nu) / ( (1 + _nu) * (1 - 2*_nu) );
    }

    // define getters for the material properties
    const std::string& name() const { return _name; }
    std::string toString() const 
    {
        // write material information to file
        return "Elastic Material '" + name() + "':\n\tDensity: " + std::to_string(density()) + "\n\tE: " + std::to_string(E()) + 
        "\n\tnu: " + std::to_string(nu()) + "\n\tmu: " + std::to_string(mu()) + "\n\tlambda: " + std::to_string(lambda());
    }
    Real density() const { return _density; }
    Real E() const { return _E; }
    Real nu() const { return _nu; }
    Real mu() const { return _mu; }
    Real lambda() const { return _lambda; }
    Real muS() const { return _mu_s; }
    Real muK() const { return _mu_k; }

    Real specificHeat() const { return _c; }
    Real thermalConductivity() const { return _k; }
    Real electricalConductivity() const { return _sigma; }

    protected:
    /** Name of the material */
    std::string _name;
    /** Density of the material */
    Real _density;
    /** Elastic modulus of the material */
    Real _E;
    /** Poisson's ratio of the material */
    Real _nu;
    /** Lame's first parameter */
    Real _lambda;
    /**  Lame's second parameter */
    Real _mu;

    /** Coefficient of static friction */
    Real _mu_s;
    /** Coefficient of kinetic friction */
    Real _mu_k;

    /** specific heat capacity. */
    Real _c;
    /** thermal conductivity */
    Real _k;
    /** electrical conductivity */
    Real _sigma;

};

#endif // __ELASTIC_MATERIAL_HPP