# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


"""
    module Modia3D.Shapes

Visual elements used for animation. The visual elements are passed to an
external renderer. Currently, the (free) community edition and the
(commercial) professional editions of the DLR Visualization library are supported
(not all visualization elements of this library are yet interfaced to Modia3D).
Modia3D is designed so that other renderers can be supported as well.

Visualization elements with shape and visualization material:

![VisuElements](../../resources/images/VisuElementsWith.jpg)


Visualization elements that do not have a visualization material:

![VisuElementsWithout](../../resources/images/VisuElementsWithout.jpg)

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
```
"""
module Shapes

using  StaticArrays
import Modia3D
import Modia3D.Basics
import Modia3D.Frames

import JSON


# Colors and material
export rgb, defaultColor, colorPalette, visualMaterialPalette

# Visual elements
export changeDimensionsBox
export Screen, XY_Plane, XZ_Plane, YZ_Plane, Left, Right, Center


# Concave properties
export ConcaveProperties

# Solid geometries
export Solid
export volume, centroid, bottomArea, topArea, longestEdge, lengthGeo, inertiaMatrix

export supportPoint_Sphere, supportPoint_Ellipsoid, supportPoint_Box, supportPoint_Cylinder, supportPoint_Cone, supportPoint_Capsule, supportPoint_Beam, supportPoint_FileMesh

export supportPoint_i_Box, supportPoint_i_Cylinder, supportPoint_i_Cone, supportPoint_i_Capsule,supportPoint_i_Beam, supportPoint_i_Ellipsoid, supportPoint_i_FileMesh

# Solid materials
export MassProperties, SolidMaterial

export MassPropertiesFromShape, MassPropertiesFromShapeAndMass

export solidMaterial, solidMaterialPalette
export regularize, resultantCoefficientOfRestitution, resultantDampingCoefficient

export rereadContactPairMaterialFromJSON

# Observer materials

export contactPairMaterialPalette, ElasticContactPairMaterial, TwoNamesKey

export NoContactPairMaterial, ObserverContactPairMaterial, ImpulseContactPairMaterial, WheelRailContactPairMaterial

export rotateAxis2x, rotateAxis2y, rotateAxis2z
export getShapeKind

# for identifying shapes at compile time
export ShapeKind, UndefinedShapeKind, SphereKind, EllipsoidKind, BoxKind, CylinderKind, ConeKind, CapsuleKind, BeamKind, FileMeshKind, CoordinateSystemKind, GridKind, SpringKind, GearWheelKind, ModelicaKind, TextKind

@enum ShapeKind UndefinedShapeKind SphereKind EllipsoidKind BoxKind CylinderKind ConeKind CapsuleKind BeamKind FileMeshKind CoordinateSystemKind GridKind SpringKind GearWheelKind ModelicaKind TextKind

include("color.jl")
include("visualMaterial.jl")
include("text.jl")
include("geometry.jl")
include("shape.jl")
include("visual.jl")
include("concaveProperties.jl")
include("inertiaTensorAndVolume.jl")
include("computePropertiesFileMes.jl")
include("boundingBoxes.jl")
include("utilities.jl")
include("solidMaterial.jl")
include("massProperties.jl")
include("contactPairMaterials.jl")
include("setCollisionSmoothingRadius.jl")
include("solid.jl")

end
