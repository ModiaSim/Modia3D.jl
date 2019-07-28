# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
# Content:
#
#    Material properties of solids

"""
    material1 = SolidMaterial(;kwargs...)
    material2 = SolidMaterial(name)

`SolidMaterial(;kwargs...)` generates a SolidMaterial object by providing the material properties of a solid with
keyword arguments. Arguments that are not provided have value = NaN.

`SolidMaterial(name)` returns a SolidMaterial object from dictionary Modia3D.solidMaterialPalette using
the name as dictionary key.

# Arguments
- `name::AbstractString`: Name of the material (used as key in dictionary Modia3D.solidMaterialPalette)

# Keyword Arguments
- `density::Float64` in [kg/m^3]: Density, see [Wikipedia](https://en.wikipedia.org/wiki/Density).
- 'YoungsModulus::Float64` in [Pa]: Youngs's modulus, see [Wikipedia](https://en.wikipedia.org/wiki/Young%27s_modulus).
- `PoissonsRatio::Float64`: Poisson's ratio, see [Wikipedia](https://en.wikipedia.org/wiki/Poisson%27s_ratio).
- `meltingPoint::Float64` in [K]: Melting point, see [Wikipedia](https://en.wikipedia.org/wiki/Melting_point).
                                  If the material is destroyed before its melting point (e.g. wood that is burning)
                                  then `meltingPoint` is the temperature when destruction of the solid starts.
- `specificHeatCapacity::Float64` in [J/(kg.K)]: Specific heat capacity, see [Wikipedia](https://en.wikipedia.org/wiki/Heat_capacity).
- `thermalConductivity::Float64` in [W/(m.K)]: Thermal conductivity, see [Wikipedia](https://en.wikipedia.org/wiki/Thermal_conductivity) and
                                [List of thermal conductivities](https://en.wikipedia.org/wiki/List_of_thermal_conductivities)
- `linearThermalExpansionCoefficient::Float64` in [1/K]: Linear thermal expansion coefficient, see [Wikipedia](https://en.wikipedia.org/wiki/Thermal_expansion).
- `coefficientOfRestitution::Float64`: Coefficient of restitution, see [Wikipedia](https://en.wikipedia.org/wiki/Coefficient_of_restitution).
- `slidingFrictionCoefficient::Float64`: Kinetic/sliding friction coefficient, see [Wikipedia](https://en.wikipedia.org/wiki/Friction)

# Example
```julia
import Modia3D
mat1 = Modia3D.SolidMaterial(density=3000.0, YoungsModulus=2e11)
mat2 = Modia3D.SolidMaterial("Steel")    # mat2.density = 8000.0
```
"""
struct SolidMaterial
   density::Float64              # [kg/m^3]  , https://en.wikipedia.org/wiki/Density
   YoungsModulus::Float64        # [Pa]      , https://en.wikipedia.org/wiki/Young%27s_modulus
   PoissonsRatio::Float64        # []        , https://en.wikipedia.org/wiki/Poisson%27s_ratio
   meltingPoint::Float64         # [K]       , https://en.wikipedia.org/wiki/Melting_point
                                 # If the material is destroyed before its melting point (e.g. wood that is burning)
                                 # then meltingPoint is the temperature when destruction starts.
   heatCapacity::Float64         # [J/(kg.K)], https://en.wikipedia.org/wiki/Heat_capacity
   thermalConductivity::Float64  # [W/(m.K)] , https://en.wikipedia.org/wiki/Thermal_conductivity
                                 #             https://en.wikipedia.org/wiki/List_of_thermal_conductivities
   linearThermalExpansionCoefficient::Float64  # [1/K], https://en.wikipedia.org/wiki/Thermal_expansion
   coefficientOfRestitution::Float64     # [] https://en.wikipedia.org/wiki/Coefficient_of_restitution
   slidingFrictionCoefficient::Float64   # [] https://en.wikipedia.org/wiki/Friction
   rotationalFrictionCoefficient::Float64
   SolidMaterial(density, YoungsModulus, PoissonsRatio, meltingPoint, heatCapacity,
                 thermalConductivity, linearThermalExpansionCoefficient, coefficientOfRestitution, slidingFrictionCoefficient,
                 rotationalFrictionCoefficient) = new(density, YoungsModulus, PoissonsRatio, meltingPoint, heatCapacity,thermalConductivity, linearThermalExpansionCoefficient, coefficientOfRestitution, slidingFrictionCoefficient,
                 rotationalFrictionCoefficient)
end
SolidMaterial(;density=NaN, YoungsModulus=NaN, PoissonsRatio=NaN, meltingPoint=NaN,
               heatCapacity=NaN, thermalConductivity=NaN, linearThermalExpansionCoefficient=NaN,
               coefficientOfRestitution=NaN, slidingFrictionCoefficient=NaN,
               rotationalFrictionCoefficient=NaN) =
              SolidMaterial(density,YoungsModulus, PoissonsRatio, meltingPoint, heatCapacity,
                            thermalConductivity, linearThermalExpansionCoefficient,
                            coefficientOfRestitution, slidingFrictionCoefficient,
                            rotationalFrictionCoefficient)


#=
const defaultFile     = joinpath(Modia3D.path, "src", "Solid", "solidMaterials.json")
#const solidMaterialPalette = JSON.parsefile(defaultFile; dicttype=Dict{String, SolidMaterial})
const solidMaterialPalette = JSON.parsefile(defaultFile)
println("typeof(solidMaterialPalette) = ", typeof(solidMaterialPalette))


solidMaterial(name::AbstractString) = solidMaterialPalette[name]
println("solidMaterial(Aluminium) = ", solidMaterial("Aluminium"))

const mat = Dict{String,SolidMaterial}()https://github.com/JuliaIO/JSON.jl/blob/master/data/jsonchecker/fail01.json
mat["Aluminium1"] = SolidMaterial(1.0,2.0,3.0,4.0,5.0,6.0,7.0)
mat["Aluminium2"] = SolidMaterial(2.0,2.0,3.0,4.0,5.0,6.0,7.0)
import Base
Base.open(joinpath(Modia3D.path, "src", "Solid", "solidMaterials2.json"), "w") do file
  JSON.print(file, mat)
end
=#

"""
    const solidMaterialPalette

Dictionary of solid material data, see [`Modia3D.SolidMaterial`](@ref)
"""
const solidMaterialPalette = Dict{String, SolidMaterial}()

# Temporary solution
solidMaterialPalette["Steel"]     = SolidMaterial(8000.0, 2.0e11, 0.30, 1640.0, 500.0, 50.0 , 1.2e-5 , 0.7, 0.5, 0.001)
solidMaterialPalette["Aluminium"] = SolidMaterial(2700.0, 6.9e10, 0.32, 933.0,  897.0, 237.0, 2.31e-5, 0.1, 1.4, 0.001)
solidMaterialPalette["DryWood"]   = SolidMaterial( 700.0, 1.1e10, 0.4 , 570.0, 1700.0,   0.1,  5.0e-6, 0.1, 0.3, 0.002)

# http://dbkcues.ru/articles-2/collision-of-billiard-balls/?lang=en

# E = 5,4 GPA
# cor=0.8, mu_k = 0.1, mu_r = 0.025 # book [Physics for game developers, Bourg, Bywalce,  2013, p. 382]
# attention: meltingPoint, heatCapacity,thermalConductivity, linearThermalExpansionCoefficient are not true!!!
solidMaterialPalette["BilliardBall"]    = SolidMaterial(1768.0, 5.4e9, 0.34, NaN, NaN, NaN, NaN, NaN, NaN, NaN)
solidMaterialPalette["BilliardCushion"] = SolidMaterial( 700.0, 1.1e10, 0.4, NaN, NaN, NaN, NaN, NaN, NaN, NaN)
solidMaterialPalette["BilliardTable"]   = SolidMaterial( 700.0, 1.1e10, 0.4, NaN, NaN, NaN, NaN, NaN, NaN, NaN)

solidMaterial(name::AbstractString) = solidMaterialPalette[name]   # Should be removed
SolidMaterial(name::AbstractString) = solidMaterialPalette[name]








"""
    material = SolidMaterial3(;kwargs...)

Generates a SolidMaterial3 object by providing the material properties of a solid with
keyword arguments. Arguments that are not provided have value = NaN.

# Keyword Arguments
- `density` in [kg/m^3]: Density, see [Wikipedia](https://en.wikipedia.org/wiki/Density).
- `YoungsModulus` in [Pa]: Youngs's modulus, see [Wikipedia](https://en.wikipedia.org/wiki/Young%27s_modulus).
- `PoissonsRatio`: Poisson's ratio, see [Wikipedia](https://en.wikipedia.org/wiki/Poisson%27s_ratio).
- `meltingPoint` in [K]: Melting point, see [Wikipedia](https://en.wikipedia.org/wiki/Melting_point).
                         If the material is destroyed before its melting point (e.g. wood that is burning)
                        then `meltingPoint` is the temperature when destruction of the solid starts.
- `specificHeatCapacity` in [J/(kg.K)]: Specific heat capacity, see [Wikipedia](https://en.wikipedia.org/wiki/Heat_capacity).
- `thermalConductivity` in [W/(m.K)]: Thermal conductivity, see [Wikipedia](https://en.wikipedia.org/wiki/Thermal_conductivity) and
                                [List of thermal conductivities](https://en.wikipedia.org/wiki/List_of_thermal_conductivities)
- `linearThermalExpansionCoefficient::Float64` in [1/K]: Linear thermal expansion coefficient, see [Wikipedia](https://en.wikipedia.org/wiki/Thermal_expansion).

# Example
```julia
import Modia3D
mat1 = Modia3D.SolidMaterial3(density=3000.0, YoungsModulus=2e11)
```
"""
mutable struct SolidMaterial3
   density::Float64                            # [kg/m^3]  , https://en.wikipedia.org/wiki/Density
   YoungsModulus::Float64                      # [Pa]      , https://en.wikipedia.org/wiki/Young%27s_modulus
   PoissonsRatio::Float64                      # []        , https://en.wikipedia.org/wiki/Poisson%27s_ratio
   meltingPoint::Float64                       # [K]       , https://en.wikipedia.org/wiki/Melting_point
                                               #             If the material is destroyed before its melting point (e.g. wood that is burning)
                                               #             then meltingPoint is the temperature when destruction starts.
   specificHeatCapacity::Float64               # [J/(kg.K)], https://en.wikipedia.org/wiki/Heat_capacity
   thermalConductivity::Float64                # [W/(m.K)] , https://en.wikipedia.org/wiki/Thermal_conductivity
                                               #             https://en.wikipedia.org/wiki/List_of_thermal_conductivities
   linearThermalExpansionCoefficient::Float64  # [1/K], https://en.wikipedia.org/wiki/Thermal_expansion
end
SolidMaterial3(; density=NaN,
                 YoungsModulus=NaN,
                 PoissonsRatio=NaN,
                 meltingPoint=NaN,
                 specificHeatCapacity=NaN,
                 thermalConductivity=NaN,
                 linearThermalExpansionCoefficient=NaN) =
              SolidMaterial3(density, YoungsModulus, PoissonsRatio, meltingPoint, specificHeatCapacity,
                             thermalConductivity, linearThermalExpansionCoefficient)


#readSolidMaterialsDict() = Modia3D.readDictOfStructsFromJSON( joinpath(Modia3D.path, "src", "Solids", "solidMaterials.json"),
#                                                              SolidMaterial3 )

"""
    readSolidMaterial3FromJSON(fileName)

Read a JSON file consisting of a dictionary of SolidMaterial3 instances from `fileName` and
return a `Dict{String, SolidMaterial3}` dictionary.
"""
readSolidMaterial3FromJSON(fileName::AbstractString) = Basics.readDictOfStructsFromJSON(fileName, SolidMaterial3)


"""
    const solidMaterialPalette3

Dictionary of solid material data, see [`Modia3D.SolidMaterial3`](@ref)
"""
const solidMaterialPalette3 = readSolidMaterial3FromJSON( joinpath(Modia3D.path, "palettes", "solidMaterials.json") )
