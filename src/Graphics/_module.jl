# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


"""
    module Modia3D.Graphics

Visual elements used for animation. The visual elements are passed to an
external renderer. Currently, the (free) community edition and the
(commercial) professional editions of the DLR Visualization library are supported
(not all visualization elements of this library are yet interfaced to Modia3D).
Modia3D is designed so that other renderers can be supported as well.

Visualization elements with geometry and visualization material:

![VisuElements](../../resources/images/VisuElementsWith.jpg)


Visualization elements that do not have a visualization material:

![VisuElementsWithout](../../resources/images/VisuElementsWithout.jpg)

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
```
"""
module Graphics

# Colors and material
export rgb, defaultColor, Material, colorPalette

# Visual elements
# (note: Pipe cannot be exported, due to a conflict with Base.Pipe)
export Sphere, Ellipsoid, Box, Cylinder, Capsule, Beam, Cone
export Spring, GearWheel, CoordinateSystem, Grid, FileMesh
export changeDimensionsBox
export Font, TextShape, Screen, XY_Plane, XZ_Plane, YZ_Plane, Left, Right, Center



using  StaticArrays
import Modia3D
import Modia3D.Basics

include("color.jl")
include("material.jl")
include("geometry.jl")
include("text.jl")

end
