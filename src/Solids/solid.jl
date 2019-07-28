# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids(Modia3D/Solids/_module.jl)
#


"""
    solid = Modia3D.Solid([geo | nothing],
                          [solidMaterialName | solidMaterial | mass |
                           massProperties    | nothing] = nothing,
                          [material = Modia3D.Material() | nothing];
                          contactMaterial=nothing)

Generate a new (rigid) solid with optional solid geometry, mass, visualization and collision properties
A solid can be associated to a [`Modia3D.Object3D`](@ref).

# Arguments
- `geo::Union{Modia3D.AbstractSolidGeometry,NOTHING}`: Optional solid geometry object
  (such as `Modia3D.SolidSphere,.SolidBox,.SolidFileMesh`).
- Mass properties (mass, center of mass, inertia matrix) of geo are computed by one of:
  * `solidMaterialName::AbstractString`: Name of a solid material defined in
    dictionary `Modia3D.solidMaterialPalette` (computed by geo and density of solid material)
  * `solidMaterial::Modia3D.SolidMaterial`: Solid material properties object
    (computed by geo and solidMaterial.density)
  * `mass::Number`: Mass in kg (computed by geo and mass).
  * `massProperties::Modia3D.MassProperties`: Mass properties (mass, center of mass, inertia matrix)
    are explicitly given.
  * `nothing`: geo has no mass.
- `material::Union{`[`Modia3D.Material`](@ref)`,NOTHING}`: Visualization material of `geo`.
   If `material=nothing`, geo is not shown in the visualization.
- `contactMaterial::Union{Modia3D.AbstractContactMaterial,NOTHING}`: Contact material of `geo`.
   If `contactMaterial=nothing`, no collision handling takes place for geo.

![Solids](../../resources/images/SolidGeos.jpg)

# Examples
```julia
import Modia3D

sbox  = Modia3D.SolidBox(1.0,2.0,3.0)
smat  = Modia3D.SolidMaterial(density = 2700)
vmat  = Modia3D.Material(color="Blue", transparency=0.5)
cmat  = Modia3D.ContactMaterialElastic(c=1e5, d=100)
massProperties = Modia3D.MassProperties(m=0.1, Ixx=1.0, Iyy=2.0, Izz=3.0)

solid1 = Modia3D.Solid(sbox, "Aluminium", vmat)
solid2 = Modia3D.Solid(sbox, smat       , vmat)
solid3 = Modia3D.Solid(sbox, 2.1        , vmat )
solid4 = Modia3D.Solid(sbox, nothing; contactMaterial=cmat)
solid5 = Modia3D.Solid(Modia3D.SolidSphere(0.1), massProperties, vmat; contactMaterial=cmat)
solid6 = Modia3D.Solid(nothing, massProperties)
```
"""
struct Solid <: Modia3D.AbstractObject3Ddata
   geo::Union{Modia3D.AbstractSolidGeometry,NOTHING}
   massProperties::Union{Solids.MassProperties,NOTHING}
   material::Union{Graphics.Material,NOTHING}
   contactMaterial::Union{String,Modia3D.AbstractContactMaterial,NOTHING}

   function Solid(geo::Union{Modia3D.AbstractSolidGeometry,NOTHING},
                  massProperties::Union{Solids.MassProperties,Number,AbstractString,Solids.SolidMaterial,NOTHING} = nothing,
                  material::Union{Graphics.Material,NOTHING} = Graphics.Material();
                  contactMaterial="")::Solid
      if typeof(massProperties) == NOTHING
         solid = new(geo, nothing, material, contactMaterial)
      else
         massprop::Solids.MassProperties = typeof(massProperties) == Solids.MassProperties ? massProperties :
                                                                     Solids.MassProperties(geo,massProperties)
         solid = new(geo, massprop, material, contactMaterial)
      end
      return solid
   end

   function Solid(geo::Vector{Modia3D.Solids.SolidFileMesh},
                  massProperties::Union{Solids.MassProperties,Number,AbstractString,Solids.SolidMaterial,NOTHING},
                  material::Union{Graphics.Material,NOTHING} = Graphics.Material();
                  contactMaterial="")::Solid
     solids = Vector{Solid}()
     for geoParts in geo
       push!(solids, Solid(geoParts, massProperties, material; contactMaterial=contactMaterial))
     end
     return solids
   end
end

#=
function canCollide(data::Solid)
  println("bin im canCollide Solid")
  return typeof(data.contactMaterial) != NOTHING && typeof(data.geo) != NOTHING
end
=#

function JSON.show_json(io::JSON.StructuralContext,
                         s::JSON.CommonSerialization, solid::Solid)
   JSON.begin_object(io)
      JSON.show_pair(io, s, "geo"             , solid.geo)
      JSON.show_pair(io, s, "massProperties"  , solid.massProperties)
      JSON.show_pair(io, s, "material"        , solid.material)
      JSON.show_pair(io, s, "contactMaterial" , solid.contactMaterial)
   JSON.end_object(io)
end



struct SolidWithConvexDecomposition
   solid::Solid                         # Solid with solid.geo = SolidFileMesh, mass properties and no contact material
   convexDecomposition::Vector{Solid}   # Vector of solids containing an approximate convex decomposition of solid.geo, no mass properties and contact material
   names::Vector{AbstractString}        # Names of the convex decompositions

   function SolidWithConvexDecomposition(
                  geo::SolidFileMesh,
                  massProperties::Union{Solids.MassProperties,Number,AbstractString,Solids.SolidMaterial,NOTHING},
                  material::Union{Graphics.Material,NOTHING} = Graphics.Material(),
                  convexDecompositionMaterial::Union{Graphics.Material,NOTHING} = Graphics.Material();
                  contactMaterial::Union{String,Modia3D.AbstractContactMaterial,NOTHING} = "")::SolidWithConvexDecomposition

      # Construct solid of SolidFileMesh
      if typeof(massProperties) == NOTHING
         solid = Solid(geo, nothing, material)
      else
         massprop::Solids.MassProperties = typeof(massProperties) == Solids.MassProperties ? massProperties :
                                                                     Solids.MassProperties(geo,massProperties)
         solid = Solid(geo, massprop, material)
      end

      # Construct solids of the approximate convex decomposition of SolidFileMesh
      convexDecompositionDirectory = joinpath(dirname(geo.filename),"convexSolids_" * basename(geo.filename))
      contentDir = readdir(convexDecompositionDirectory)
      convexDecomposition = Vector{Solid}()
      names = Vector{AbstractString}()
      for name in contentDir
         (head,ext) = splitext(name)
         if ext == ".obj"
            push!(convexDecomposition, Solid(SolidFileMesh(joinpath(convexDecompositionDirectory, name);
                                                           scaleFactor              = geo.scaleFactor,
                                                           useGraphicsMaterialColor = geo.useGraphicsMaterialColor,
                                                           smoothNormals            = geo.smoothNormals),
                                             nothing, convexDecompositionMaterial; contactMaterial = contactMaterial))
            push!(names, head)
         end
      end

      # Generate instance
      new(solid, convexDecomposition, names)
   end
end
