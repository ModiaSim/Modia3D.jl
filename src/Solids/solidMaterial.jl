# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module 
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
# Content:
#
#    Material properties of solids

struct SolidMaterial
   density::Float64              # [kg/m^3]  , https://en.wikipedia.org/wiki/Density
   YoungsModulus::Float64        # [Pa]      , https://en.wikipedia.org/wiki/Young%27s_modulus
   PoissonsRatio::Float64        # []        , https://en.wikipedia.org/wiki/Poisson%27s_ratio
   meltingPoint::Float64         # [K]       , https://en.wikipedia.org/wiki/Melting_point
   heatCapacity::Float64         # [J/(kg.K)], https://en.wikipedia.org/wiki/Heat_capacity
   thermalConductivity::Float64  # [W/(m.K)] , https://en.wikipedia.org/wiki/Thermal_conductivity
                                 #             https://en.wikipedia.org/wiki/List_of_thermal_conductivities
   linearThermalExpansionCoefficient::Float64  # [1/K], https://en.wikipedia.org/wiki/Thermal_expansion
end
SolidMaterial(;density=NaN, YoungsModulus=NaN, PoissonsRatio=NaN, meltingPoint=NaN,
               heatCapacity=NaN, thermalConductivity=NaN, linearThermalExpansionCoefficient=NaN) =
              SolidMaterial(density,YoungsModulus, PoissonsRatio, meltingPoint, heatCapacity,
                            thermalConductivity, linearThermalExpansionCoefficient)


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

const solidMaterialPalette = Dict{String, SolidMaterial}()

# Temporary solution
solidMaterialPalette["Aluminium"] = SolidMaterial(2700.0, 6.9e10, 0.32, 933.0,  897.0, 237.0, 2.31e-5)
solidMaterialPalette["Steel"]     = SolidMaterial(7900.0, 2.0e11, 0.29, 1700.0, 446.0,  50.2, 17.2e-6)
solidMaterialPalette["DryWood"]   = SolidMaterial( 700.0, 1.1e10, 0.4 , NaN  , 1700.0,   0.1, 5.0e-6)

solidMaterial(name::AbstractString) = solidMaterialPalette[name]


