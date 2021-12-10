# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Solids/_module.jl)
#


# MassProperties consisting of mass m, center of mass rCM and inertia tensor I
struct MassProperties{F} <: Modia3D.AbstractMassProperties
    m::Float64                 # mass [kg]
    rCM::Frames.Vector3D       # position vector from object3d frame to center of mass resolved in object3d frame [m]
    I::SMatrix{3,3,Float64,9}  # inertia matrix w.r.t. center of mass resolved in object3d frame [kg.m^2]

    #---------------- different constructors for MassProperties -----------------
    # Constructor 0: takes mass, centerOfMass and inertiaMatrix as input values
    function MassProperties{F}(mass::Number, centerOfMass::AbstractVector, inertiaMatrix::AbstractMatrix) where {F}
        @assert(mass > 0.0)
        new(mass, centerOfMass, inertiaMatrix)
    end
end

MassProperties(args...; kwargs...) = MassProperties{Float64}(args...; kwargs...)


struct MassPropertiesFromShape{F} <: Modia3D.AbstractMassProperties
    function MassPropertiesFromShape{F}() where {F}
        new()
    end
end
MassPropertiesFromShape(args...; kwargs...) = MassPropertiesFromShape{Float64}(args...; kwargs...)

struct MassPropertiesFromShapeAndMass{F} <: Modia3D.AbstractMassProperties
    mass::Number   # mass in [kg]

    function MassPropertiesFromShapeAndMass{F}(;mass::Number=1.0) where {F}
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
#                --> if nothing special is defined it takes predefined values (= zero values)
MassProperties{F}(; mass::Number=0.0, centerOfMass=Modia3D.ZeroVector3D,
               Ixx::Number=0.0, Iyy::Number=0.0, Izz::Number=0.0,
               Ixy::Number=0.0, Ixz::Number=0.0, Iyz::Number=0.0) where {F} =
                  MassProperties{F}(mass, centerOfMass, [Ixx Ixy Ixz; Ixy Iyy Iyz; Ixz Iyz Izz])
# Constructor b: shape and mass is given, center of mass and inertia tensor is
#                calculated via shape --> constructor 0 is called
MassProperties{F}(shape::Modia3D.AbstractGeometry, mass::Number) where {F} =
                     MassProperties{F}(mass, centroid(shape), inertiaMatrix(shape,mass))
# Constructor c: shape and material is given, mass is computed via volume of
#                shape and density --> constructor b is called
MassProperties{F}(shape::Modia3D.AbstractGeometry, material::SolidMaterial) where {F} =
                     MassProperties{F}(shape, material.density*volume(shape))
# Constructor d: shape and materialName is given, material must be defined in
#                solidMaterialPalette --> constructor c is called
MassProperties{F}(shape::Modia3D.AbstractGeometry, materialName::AbstractString) where {F} =
                     MassProperties{F}(shape, solidMaterialPalette[materialName])


createMassProperties(::Type{F}, massProperties::MassProperties, shape, solidMaterial) where {F} = massProperties

createMassProperties(::Type{F}, massProperties::Union{Number, SolidMaterial, AbstractString}, shape::Modia3D.AbstractGeometry, solidMaterial) where {F} = MassProperties{F}(shape, massProperties)

# compute mass properties from shape and material
function createMassProperties(::Type{F}, massProperties::Union{MassPropertiesFromShape,Nothing}, shape::Modia3D.AbstractGeometry, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where {F}
    if isnothing(solidMaterial)
        error("It is not possible to compute mass properties (MassPropertiesFromShape = ", massProperties,") for shape = ", shape , " because no solidMaterial is defined.")
    else
        return Modia3D.MassProperties{F}(shape, solidMaterial)
    end
end

# compute mass properties from shape and mass
function createMassProperties(::Type{F}, massProperties::MassPropertiesFromShapeAndMass, shape::Modia3D.AbstractGeometry, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where {F}
    return Modia3D.MassProperties{F}(shape, massProperties.mass)
end

function createMassProperties(::Type{F}, massProperties::MassPropertiesFromShape, shape::Nothing, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where {F}
    error("It is not possible to compute mass properties (MassPropertiesFromShape = ", massProperties,") because no shape is defined.")
end

function createMassProperties(::Type{F}, massProperties::MassPropertiesFromShapeAndMass, shape::Nothing, solidMaterial::Union{AbstractString,SolidMaterial,Nothing}) where {F}
    error("It is not possible to compute mass properties (MassPropertiesFromShapeAndMass) if no shape is defined.")
end
