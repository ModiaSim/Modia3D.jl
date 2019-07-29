# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

"""
    module Modia3D.Solids

Module `Solids` provides data structures and operations for solids, so objects that have a volume and
properties associated with the volume.
Solid parts can be associated with a [`Modia3D.Object3D`](@ref).
They are defined with struct [`Modia3D.Solid`](@ref) consisting of an **optional** solid geometry:

![Solids](../../resources/images/SolidGeos.jpg)

and other **optional** properties:

- mass propreties (defined by geometry+material-name, geometry+density, or directly defined mass properties),
- contact material (for elastic response calculation),
- visualization material (for visualization, see below).

Since the solid geometry itself is optional, it is possible to just define a
coordinate system with associated mass, center of mass and inertia matrix.

The following functions are provided for a solid geometry `geo::Modia3D.AbstractSolidGeometry` that
is associated with an Object3D `object3D`:
- [`Modia3D.volume`](@ref)`(geo)` returns the volume of `geo`.
- [`Modia3D.centroid`](@ref)`(geo)` returns the centroid of `geo` (= center of mass for uniform density).
- [`Modia3D.inertiaMatrix`](@ref)`(geo, mass)` returns the inertia matrix of `geo`.
- [`Modia3D.boundingBox!`](@ref)`(geo, <other arguments>)` returns the Axis Aligned Bounding Box of `geo`.
- [`Modia3D.supportPoint`](@ref)`(geo, <other arguments>)` returns the support point of `geo` along the desired direction.
- isVisible(object3D, renderer),
- dataHasMass(object3D),
- canCollide(object3D),
- and other functions.

Other functions in module `Solids`:
- [`Modia3D.SolidMaterial`](@ref)(name) returns the solid properties of material `name`.
- [`Modia3D.ElasticContactMaterial`](@ref)(name) returns the elastic contact properties of material `name`.

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module Solids

# Concave properties
export ConcaveProperties

# Solid geometries
export Solid, SolidFileMesh, SolidWithConvexDecomposition
export SolidSphere, SolidEllipsoid, SolidBox, SolidCylinder, SolidCapsule, SolidBeam, SolidCone, SolidPipe
export volume, centroid, bottomArea, topArea, longestEdge, lengthGeo, inertiaMatrix, boundingBox!, supportPoint
# export canCollide

# Solid materials
export InternalMassProperties, MassProperties, SolidMaterial, ContactMaterialElastic, dummyMassProperties
export solidMaterial, solidMaterialPalette, defaultContactMaterial
export regularize, resultantCoefficientOfRestitution, resultantDampingCoefficient
export solidMaterialPairsPalette, CommonCollisionProperties, getCommonCollisionProperties

export SolidMaterial2, solidMaterialPalette2
export SolidMaterial3, solidMaterialPalette3

export ElasticContactMaterial, ElasticContactMaterialFromMaterialData, ElasticContactMaterialFromMaterialName
export ElasticContactMaterial2   # just temporarily
export ElasticContactPairMaterial2
export contactPairMaterialPalette, ContactPairMaterial, TwoNamesKey

using StaticArrays
import JSON
import Modia3D
import Modia3D.Basics
import Modia3D.Graphics
import ModiaMath

const NOTHING = Nothing


include("concaveProperties.jl")
include("geometry.jl")
include("solidMaterial.jl")
include("massProperties.jl")
include("contactMaterial.jl")
include("contactPairMaterials.jl")
include("solid.jl")

end
