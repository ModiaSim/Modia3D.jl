# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Solids/_module.jl)
#


# MassProperties consisting of mass m, center of mass rCM and inertia tensor I
struct MassProperties{F <: Modia3D.VarFloatType} <: Modia3D.AbstractMassProperties
    m::F                 # mass [kg]
    rCM::SVector{3,F}       # position vector from object3d frame to center of mass resolved in object3d frame [m]
    I::SMatrix{3,3,F,9}  # inertia matrix w.r.t. center of mass resolved in object3d frame [kg.m^2]

    #---------------- different constructors for MassProperties -----------------
    # Constructor 0: takes mass, centerOfMass and inertiaMatrix as input values
    function MassProperties{F}(mass::Number, centerOfMass::AbstractVector, inertiaMatrix::AbstractMatrix) where F <: Modia3D.VarFloatType
        @assert(mass >= 0.0)
        new(mass, centerOfMass, inertiaMatrix)
    end
end

MassProperties(args...; kwargs...) = MassProperties{Float64}(args...; kwargs...)


struct MassPropertiesFromShape{F <: Modia3D.VarFloatType} <: Modia3D.AbstractMassProperties
    function MassPropertiesFromShape{F}() where F <: Modia3D.VarFloatType
        new()
    end
end
MassPropertiesFromShape(args...; kwargs...) = MassPropertiesFromShape{Float64}(args...; kwargs...)

struct MassPropertiesFromShapeAndMass{F <: Modia3D.VarFloatType} <: Modia3D.AbstractMassProperties
    mass::F   # mass in [kg]

    function MassPropertiesFromShapeAndMass{F}(; mass=F(1.0) ) where F <: Modia3D.VarFloatType
        @assert(mass >= 0.0)
        new(mass)
    end
end
MassPropertiesFromShapeAndMass(args...; kwargs...) = MassPropertiesFromShapeAndMass{Float64}(args...; kwargs...)

function Base.show(io::IO, mp::MassProperties)
    print(io, "mass = ", mp.m,
            ", centerOfMass = ", mp.rCM,
            ", Ixx = ", mp.I[1,1],
            ", Iyy = ", mp.I[2,2],
            ", Izz = ", mp.I[3,3],
            ", Ixy = ", mp.I[1,2],
            ", Ixz = ", mp.I[1,3],
            ", Iyz = ", mp.I[2,3])
end

# Constructor a: mass, centerOfMass and entries of inertia tensor are optional
"""
    massProps = MassProperties(;
        mass::Number = 0,
        centerOfMass::Real = [0.0, 0.0, 0.0],
        Ixx::Real = 0.0,
        Iyy::Real = 0.0,
        Izz::Real = 0.0,
        Ixy::Real = 0.0,
        Ixz::Real = 0.0,
        Iyz::Real = 0.0)

Return mass properties of a [`Solid`](@ref).

# Arguments
- `mass`: Mass of the solid [kg].
- `centerOfMass`: Position vector from the Object3D frame to the center of mass, resolved in Object3D frame [m].
- `Ixx`: x-moment of inertia [kg.m^2].
- `Iyy`: y-moment of inertia [kg.m^2].
- `Izz`: z-moment of inertia [kg.m^2].
- `Ixy`: xy-product of inertia [kg.m^2].
- `Ixz`: zx-product of inertia [kg.m^2].
- `Iyz`: yz-product of inertia [kg.m^2].

# Notes
- The moments and products of inertia are defined with respect to the center of mass, resolved in the Object3D frame.
- The products of inertia are defined in the [physical correct representation](https://en.wikipedia.org/wiki/Moment_of_inertia#Definition_2) (i.e. with negative sign) and are therefore directly used as off-diagonal inertia tensor elements.
- Attention: [Some CAE applications use products of inertia defined with inversed sign](https://en.wikipedia.org/wiki/Moment_of_inertia#Alternate_inertia_convention).
"""
MassProperties{F}(; mass::Number=F(0.0), centerOfMass=Modia3D.ZeroVector3D(F),
                    Ixx::Number=F(0.0), Iyy::Number=F(0.0), Izz::Number=F(0.0),
                    Ixy::Number=F(0.0), Ixz::Number=F(0.0), Iyz::Number=F(0.0)) where F <: Modia3D.VarFloatType =
    MassProperties{F}(mass, centerOfMass, [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz])
# Constructor b: shape and mass is given, center of mass and inertia tensor is
#                calculated via shape --> constructor 0 is called
MassProperties{F}(shape::Modia3D.AbstractGeometry, mass::Number) where F <: Modia3D.VarFloatType =
    MassProperties{F}(F(mass), centroid(shape), inertiaMatrix(shape,F(mass)))
# Constructor c: shape and material is given, mass is computed via volume of
#                shape and density --> constructor b is called
MassProperties{F}(shape::Modia3D.AbstractGeometry, material::SolidMaterial) where F <: Modia3D.VarFloatType =
    MassProperties{F}(shape, material.density*volume(shape))
# Constructor d: shape and materialName is given, material must be defined in
#                solidMaterialPalette --> constructor c is called
MassProperties{F}(shape::Modia3D.AbstractGeometry, materialName::AbstractString) where F <: Modia3D.VarFloatType =
    MassProperties{F}(shape, solidMaterialPalette[1][materialName])


createMassProperties(::Type{F}, massProperties::MassProperties, shape, solidMaterial) where F <: Modia3D.VarFloatType = massProperties

createMassProperties(::Type{F}, massProperties::Union{Number, SolidMaterial, AbstractString}, shape::Modia3D.AbstractGeometry, solidMaterial) where F <: Modia3D.VarFloatType = MassProperties{F}(shape, massProperties)

# compute mass properties from shape and material
function createMassProperties(::Type{F}, massProperties::Union{MassPropertiesFromShape,Nothing}, shape::Modia3D.AbstractGeometry, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where F <: Modia3D.VarFloatType
    if isnothing(solidMaterial)
        error("It is not possible to compute mass properties (MassPropertiesFromShape = ", massProperties,") for shape = ", shape , " because no solidMaterial is defined.")
    else
        return Modia3D.Shapes.MassProperties{F}(shape, solidMaterial)
    end
end

# compute mass properties from shape and mass
function createMassProperties(::Type{F}, massProperties::MassPropertiesFromShapeAndMass, shape::Modia3D.AbstractGeometry, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where F <: Modia3D.VarFloatType
    return Modia3D.Shapes.MassProperties{F}(shape, massProperties.mass)
end

function createMassProperties(::Type{F}, massProperties::MassPropertiesFromShape, shape::Nothing, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where F <: Modia3D.VarFloatType
    error("It is not possible to compute mass properties (MassPropertiesFromShape = ", massProperties,") because no shape is defined.")
end

function createMassProperties(::Type{F}, massProperties::MassPropertiesFromShapeAndMass, shape::Nothing, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where F <: Modia3D.VarFloatType
    error("It is not possible to compute mass properties (MassPropertiesFromShapeAndMass) if no shape is defined.")
end
