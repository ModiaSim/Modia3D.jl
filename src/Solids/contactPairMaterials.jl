# License for this file: MIT (expat)
# Copyright 2017-2019, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids(Modia3D/Solids/_module.jl)
#


"""
    material = ElasticContactPairMaterial(;coefficientOfRestitution=0.0, slidingFrictionCoefficient=0.0,
                                           rotationalResistanceCoefficient=0.0, vsmall=0.01, wsmall=0.01)

Generates an `ElasticContactPairMaterial` object by providing the material properties of
two objects that are in contact to each other.

# Keyword Arguments
- coefficientOfRestitution: Coefficient of restitution between two objects (=0: inelastic ... =1: fully elastic).
- slidingFrictionCoefficient: Kinetic/sliding friction force coefficent between two objects (`>= 0.0`).
- rotationalResistanceCoefficient: Rotational resistance torque coefficient between two objects (`>= 0.0`).
  Its effect is that the contact torque is computed to reduce the relative angular velocity between two objects.
  For a ball, `rotationalResistanceCoefficient` is the (standard) rolling resistance coefficient.
- `vsmall` in [m/s]: Used for regularization when computing the unit vector in direction of
                     the relative tangential velocity to avoid a division by zero.
- `wsmall` in [rad/s]: Used for regularization when computing the unit vector in direction
                       of the relative angular velocity to avoid a division by zero.

# Example
```julia
import Modia3D
mat = Modia3D.ElasticContactPairMaterial(coefficientOfRestitution=0.5)
```
"""
mutable struct ElasticContactPairMaterial <: Modia3D.AbstractContactPairMaterial
    coefficientOfRestitution::Float64
    slidingFrictionCoefficient::Float64
    rotationalResistanceCoefficient::Float64
    vsmall::Float64
    wsmall::Float64

    function ElasticContactPairMaterial(; coefficientOfRestitution=0.0,
                                          slidingFrictionCoefficient=0.0,
                                          rotationalResistanceCoefficient=0.0,
                                          vsmall=0.01,
                                          wsmall=0.01)
        @assert(coefficientOfRestitution >= 0.0)
        @assert(coefficientOfRestitution <= 1.0)
        @assert(slidingFrictionCoefficient >= 0.0)
        @assert(rotationalResistanceCoefficient >= 0.0)
        new(coefficientOfRestitution, slidingFrictionCoefficient, rotationalResistanceCoefficient,
            vsmall, wsmall)
    end
end


"""
    material = combineContactPairMaterials(material1, material2)

Return a new `ElasticContactPairMaterial` object from the mean values of the ElasticContactPairMaterial
objects `material1` and `material2`.
"""
function combineContactPairMaterials(mat1::ElasticContactPairMaterial, mat2::ElasticContactPairMaterial)::ElasticContactPairMaterial
    mat = ElasticContactPairMaterial()
    mat.coefficientOfRestitution        = (mat1.coefficientOfRestitution        + mat2.coefficientOfRestitution)/2
    mat.slidingFrictionCoefficient      = (mat1.slidingFrictionCoefficient      + mat2.slidingFrictionCoefficient)/2
    mat.rotationalResistanceCoefficient = (mat1.rotationalResistanceCoefficient + mat2.rotationalResistanceCoefficient)/2
    mat.vsmall = (mat1.vsmall + mat2.vsmall)/2
    mat.wsmall = (mat1.wsmall + mat2.wsmall)/2
    return mat
end


struct TwoNamesKey
    name1::String
    name2::String
    function TwoNamesKey(name1::String, name2::String)
        if name1 < name2
            return new(name1, name2)
        else
            return new(name2, name1)
        end
    end
end

Base.:isequal(keyA::TwoNamesKey, keyB::TwoNamesKey) = keyA.name1 == keyB.name1 && keyA.name2 == keyB.name2


"""
    readContactPairMaterialFromJSON(fileName)

Read a JSON file consisting of a dictionary of ContactPairMaterial instances from `fileName` and
return a `Dict{String, ContactPairMaterial}` dictionary.
"""
function readContactPairMaterialFromJSON(fileName::AbstractString)
    dict1 = JSON.parsefile(fileName)
    palette = Dict{TwoNamesKey, Modia3D.AbstractContactPairMaterial}()
    for (key1,value1) in dict1
        # Split a key "name1,name2" in two parts
        i = findfirst(",", key1)
        if typeof(i) == Nothing || i.start < 2 || i.start >= length(key1)
            println("... Wrong key \"", key1,"\" not included in contactPairMaterialPalette.")
        else
            key2 = TwoNamesKey( key1[1:i.start-1], key1[i.start+1:end] )
            obj  = ElasticContactPairMaterial()
            for (key3,value3) in value1
                setfield!(obj, Symbol(key3), value3)
            end
            palette[key2] = obj
        end
    end
    return palette
end


"""
    const contactPairMaterialPalette

Dictionary of contact pair material data, see [`Modia3D.ContactPairMaterial`](@ref)
"""
const contactPairMaterialPalette = readContactPairMaterialFromJSON( joinpath(Modia3D.path, "src", "Solids", "contactPairMaterials.json") )



struct NoContactPairMaterial <: Modia3D.AbstractContactPairMaterial
end


"""
    material = getContactPairMaterial(name1, name2)

Return a contact pair material object of the materials with the
names `name1` and `name2` from dictionary `Modia3D.contactPairMaterialPalette`.
"""
function getContactPairMaterial(name1::AbstractString, name2::AbstractString)::Modia3D.AbstractContactPairMaterial
    value = get(contactPairMaterialPalette, TwoNamesKey(name1,name2), NoContactPairMaterial())
    if typeof(value) == NoContactPairMaterial
        # Combination name1,name2 is not present
        value1 = get(contactPairMaterialPalette, TwoNamesKey(name1,name1), NoContactPairMaterial())
        value2 = get(contactPairMaterialPalette, TwoNamesKey(name2,name2), NoContactPairMaterial())
        if typeof(value1) == NoContactPairMaterial || typeof(value2) == NoContactPairMaterial
            error("No contact pair material (\"$name1\",\"$name2\") or\n",
                  "(\"$name1\",\"$name1\") and (\"$name2\",\"$name2\")\n",
                  "in Modia3D.contactPairMaterialPalette dictionary.")
        end
        return combineContactPairMaterials(value1, value2)
    end
    return value
end
