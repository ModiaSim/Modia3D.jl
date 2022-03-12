"""
    SolidMaterial(density=3000.0, YoungsModulus=2e11)

Generates a SolidMaterial object by providing the material properties of a [Solid] with keyword arguments.

# Arguments
Arguments that are not provided have value `NaN`.
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

"""
mutable struct SolidMaterial
   density::Float64              # [kg/m^3], https://en.wikipedia.org/wiki/Density
   YoungsModulus::Float64        # [Pa], https://en.wikipedia.org/wiki/Young%27s_modulus
   PoissonsRatio::Float64        # [],   https://en.wikipedia.org/wiki/Poisson%27s_ratio
   meltingPoint::Float64         # [K],  https://en.wikipedia.org/wiki/Melting_point
                                 #       If the material is destroyed before its melting point
                                 #       (e.g. wood that is burning) then meltingPoint is the
                                 #       temperature when destruction starts.
   specificHeatCapacity::Float64 # [J/(kg.K)], https://en.wikipedia.org/wiki/Heat_capacity
   thermalConductivity::Float64  # [W/(m.K)],  https://en.wikipedia.org/wiki/Thermal_conductivity
                                 #             https://en.wikipedia.org/wiki/List_of_thermal_conductivities
   linearThermalExpansionCoefficient::Float64  # [1/K], https://en.wikipedia.org/wiki/Thermal_expansion
end
SolidMaterial(; density=NaN,
                 YoungsModulus=NaN,
                 PoissonsRatio=NaN,
                 meltingPoint=NaN,
                 specificHeatCapacity=NaN,
                 thermalConductivity=NaN,
                 linearThermalExpansionCoefficient=NaN) =
              SolidMaterial(density, YoungsModulus, PoissonsRatio, meltingPoint, specificHeatCapacity,
                             thermalConductivity, linearThermalExpansionCoefficient)


"""
    readSolidMaterialFromJSON(fileName)

Read a JSON file consisting of a dictionary of SolidMaterial instances from `fileName` and
return a `Dict{String, SolidMaterial}` dictionary.
"""
readSolidMaterialFromJSON(fileName::AbstractString) =
         Basics.readDictOfStructsFromJSON(fileName, SolidMaterial)


"""
    const solidMaterialPalette

Dictionary of solid material data, see [`SolidMaterial`](@ref)
"""
const solidMaterialPalette =
         [readSolidMaterialFromJSON( joinpath(Modia3D.path, "palettes", "solidMaterials.json") )]        

function rereadSolidMaterialFromJSON(; file="")
    if file == ""
        file = joinpath(Modia3D.path, "palettes", "solidMaterials.json")
    end
    
    global solidMaterialPalette[1] = readSolidMaterialFromJSON(file) 
    return nothing
end
