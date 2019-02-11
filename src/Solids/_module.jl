# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

"""
    module Modia3D.Solids

Objects that have a volume and properties associated with the volume.
Solid parts can be associated with a [`Modia3D.Object3D`](@ref).
They are defined with struct [`Modia3D.Solid`](@ref) consisting of an **optional** solid geometry:

![Solids](../../resources/images/SolidGeos.jpg)

and other **optional** properties:

- mass propreties (defined by geometry+material-name, geometry+density, or directly defined mass properties),
- contact material (for elastic response calculation),
- visualization material (for visualization, see below).

Since the solid geometry itself is optional, it is possible to just define a
coordinate system with associated mass, center of mass and inertia matrix.

The following functions are provided for a solid geometry `geo` that
is associated with an Object3D `object3D`:
- volume(geo),
- centroid(geo),
- inertiaMatrix(geo, mass),
- boundingBox(geo, <other arguments>),
- supportPoint(geo, <other arguments>),
- isVisible(object3D, renderer),
- dataHasMass(object3D),
- canCollide(object3D),
- and other functions.

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
export MassProperties, SolidMaterial, ContactMaterialElastic, dummyMassProperties
export solidMaterial, solidMaterialPalette, defaultContactMaterial

using StaticArrays
import JSON
import Modia3D
import Modia3D.Basics
import Modia3D.Graphics
import ModiaMath

@static if VERSION >= v"0.7.0-DEV.2005"
    const NOTHING = Nothing
else
    const NOTHING = Void
end

include("concaveProperties.jl")
include("geometry.jl")
include("solidMaterial.jl")
include("massProperties.jl")
include("contactMaterial.jl")
include("solid.jl")

end
