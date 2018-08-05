# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
# Content:
#
#    Mass, centerOfMass, inertiaMatrix of a solid

struct MassProperties
   m::Float64                 # mass in [kg]
   rCM::ModiaMath.Vector3D    # center of mass in [m]
   I::SMatrix{3,3,Float64,9}  # inertia matrix in [kg.m^2]

   function MassProperties(mass::Number, centerOfMass::AbstractVector, inertiaMatrix::AbstractMatrix)
      @assert(mass > 0.0)
      new(mass, centerOfMass, inertiaMatrix)
   end
end

MassProperties(;m::Number=0.0, rCM=ModiaMath.ZeroVector3D,
       Ixx::Number=0.0, Iyy::Number=0.0, Izz::Number=0.0,
       Ixy::Number=0.0, Ixz::Number=0.0, Iyz::Number=0.0) = MassProperties(m, rCM, [Ixx Ixy Ixz;
                                                                                    Ixy Iyy Iyz;
                                                                                    Ixz Iyz Izz])

MassProperties{Geo<:Modia3D.AbstractSolidGeometry}(geo::Geo, mass::Number)                 = MassProperties(mass, centroid(geo), inertiaMatrix(geo,mass))

MassProperties{Geo<:Modia3D.AbstractSolidGeometry}(geo::Geo, material::SolidMaterial)      = MassProperties(geo, material.density*volume(geo))

MassProperties{Geo<:Modia3D.AbstractSolidGeometry}(geo::Geo, materialName::AbstractString) = MassProperties(geo, solidMaterialPalette[materialName])
