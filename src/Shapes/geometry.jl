# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Shapes/_module.jl)
#
# Content:
#   This file contains mutable visual geometries that are only used for animation
#   (immutable solid geometries that have a volume are defined in module Modia3D.Shapes.
#    They can be additionally used for collision handling and other purposes).

"""
    Sphere(; diameter=1.0)

Generate a new solid or visual shape representing a sphere.
The reference frame = Object3D frame is located at the center of the sphere.

# Arguments
- `diameter` defines the diameter of the sphere.
"""
mutable struct Sphere{F} <: Modia3D.AbstractGeometry
    diameter::F

    function Sphere{F}(; diameter=F(1.0) ) where {F}
        @assert(diameter >= 0.0)
        new(diameter)
    end
end
Sphere(; kwargs...) = Sphere{Float64}(; kwargs...)

"""
    Ellipsoid(; lengthX=1.0, lengthY=1.0, lengthZ=1.0)

Generate a new solid or visual shape representing an ellipsoid.
The reference frame = Object3D frame is located at the center of the ellipsoid.

# Arguments
- `lengthX` defines the length of the ellipsoid in x-direction.
- `lengthY` defines the length of the ellipsoid in y-direction.
- `lengthZ` defines the length of the ellipsoid in z-direction.
"""
mutable struct Ellipsoid{F} <: Modia3D.AbstractGeometry
    lengthX::F
    lengthY::F
    lengthZ::F

    function Ellipsoid{F}(; lengthX=F(1.0), lengthY=F(1.0), lengthZ=F(1.0) ) where {F}
        @assert(lengthX >= 0.0)
        @assert(lengthY >= 0.0)
        @assert(lengthZ >= 0.0)
        if lengthX == lengthY == lengthZ
            Sphere{F}(diameter=lengthX)
        else
            new(lengthX, lengthY, lengthZ)
        end
    end
end
Ellipsoid(; kwargs...) = Ellipsoid{Float64}(; kwargs...)

"""
    Box(; lengthX=1.0, lengthY=1.0, lengthZ=1.0)

Generate a new solid or visual shape representing a box.
The reference frame = Object3D frame is located at the center of the box.

# Arguments
- `lengthX` defines the length of the box in x-direction.
- `lengthY` defines the length of the box in y-direction.
- `lengthZ` defines the length of the box in z-direction.
"""
mutable struct Box{F} <: Modia3D.AbstractGeometry
    lengthX::F
    lengthY::F
    lengthZ::F

    function Box{F}(; lengthX=F(1.0), lengthY=F(1.0), lengthZ=F(1.0) ) where {F}
        @assert(lengthX >= 0.0)
        @assert(lengthY >= 0.0)
        @assert(lengthZ >= 0.0)
        new(lengthX, lengthY, lengthZ)
    end
end
Box(; kwargs...) = Box{Float64}(; kwargs...)

"""
    Cylinder(; axis=3, diameter=1.0, length=1.0, innerDiameter=0.0)

Generate a new solid or visual shape representing a cylinder.
The reference frame = Object3D frame is located at the center of the cylinder.

# Arguments
- `axis` defines the rotation axis of the cylinder: 1=x; 2=y; 3=z.
- `diameter` defines the diameter of the cylinder.
- `length` defines the length of the cylinder.
- `innerDiameter` defines the inner diameter of the cylinder (where `innerDiameter=0` defines a full cylinder).

# Notes
- `innerDiameter` is not supported by collision.
"""
mutable struct Cylinder{F} <: Modia3D.AbstractGeometry
    axis::Int
    diameter::F
    length::F
    innerDiameter::F

    function Cylinder{F}(; axis=3, diameter=F(1.0), length=F(1.0), innerDiameter=F(0.0) ) where {F}
        @assert(1 <= axis <= 3)
        @assert(diameter >= 0.0)
        @assert(length >= 0.0)
        @assert(0.0 <= innerDiameter < diameter)
        new(axis, diameter, length, innerDiameter)
    end
end
Cylinder(; kwargs...) = Cylinder{Float64}(; kwargs...)

"""
    Cone(; axis=3, diameter=1.0, length=1.0, topDiameter=0.0)

Generate a new solid or visual shape representing a circular cone or a frustum of a circular cone.
The reference frame = Object3D frame is located at the center of the base circle of the cone/frustum.

# Arguments
- `axis` defines the rotation axis of the cone/frustum: 1=x; 2=y; 3=z.
- `diameter` defines the diameter of the base circle of the cone/frustum.
- `length` defines the length of the cone/frustum.
- `topDiameter` defines the diameter of the top circle of the frustum (where `topDiameter=0` defines a right cone).
"""
mutable struct Cone{F} <: Modia3D.AbstractGeometry
    axis::Int
    diameter::F
    length::F
    topDiameter::F

    function Cone{F}(; axis=3, diameter=F(1.0), length=F(1.0), topDiameter=F(0.0) ) where {F}
        @assert(1 <= axis <= 3)
        @assert(diameter >= 0.0)
        @assert(length >= 0.0)
        @assert(0 <= topDiameter < diameter)
        new(axis, diameter, length, topDiameter)
    end
end
Cone(; kwargs...) = Cone{Float64}(; kwargs...)

"""
    Capsule(; axis=3, diameter=1.0, length=1.0)

Generate a new solid or visual shape representing a capsule assembled by a cylinder and two half spheres.
The reference frame = Object3D frame is located at the center of the capsule.

# Arguments
- `axis` defines the rotation axis of the capsule: 1=x; 2=y; 3=z.
- `diameter` defines the diameter of the capsule.
- `length` defines the length of the middle cylindrical part of the capsule.

# Notes
- Some versions of SimVis visualize [half ellipsoids with semi-axis length `length/2` instead of spheres with diameter `diameter`](https://github.com/ModiaSim/PrivateModia3D.jl/issues/54).
"""
mutable struct Capsule{F} <: Modia3D.AbstractGeometry
    axis::Int
    diameter::F
    length::F

    function Capsule{F}(; axis=3, diameter=F(1.0), length=F(1.0) ) where {F}
        @assert(1 <= axis <= 3)
        @assert(diameter >= 0.0)
        @assert(length >= 0.0)
        new(axis, diameter, length)
    end
end
Capsule(; kwargs...) = Capsule{Float64}(; kwargs...)

"""
    Beam(; axis=3, length=1.0, width=0.2, thickness=0.1)

Generate a new solid or visual shape representing a beam assembled by a box and two half cylinders.
The reference frame = Object3D frame is located at the center of the beam.

# Arguments
- `axis` defines the longitudinal axis of the beam: 1=x; 2=y; 3=z.
- `length` defines the box length of the beam. The total length of the beam is `length + width`.
- `width` defines the width (= cylinder diameter) of the beam.
- `thickness` defines the thickness of the beam.

The dimension directions depend on `axis` by circularly shift:

| `axis` | `length` | `width` | `thickness` |
|:------:|:--------:|:-------:|:-----------:|
|   1    |   x      |   y     |   z         |
|   2    |   y      |   z     |   x         |
|   3    |   z      |   x     |   y         |

"""
mutable struct Beam{F} <: Modia3D.AbstractGeometry
    axis::Int
    length::F
    width::F
    thickness::F

    function Beam{F}(; axis=3, length=F(1.0), width=F(0.2), thickness=F(0.1) ) where {F}
        @assert(1 <= axis <= 3)
        @assert(length >= 0.0)
        @assert(width >= 0.0)
        @assert(thickness >= 0.0)
        new(axis, length, width, thickness)
    end
end
Beam(; kwargs...) = Beam{Float64}(; kwargs...)

"""
    FileMesh(; filename::AbstractString="", scale=SVector{3,Float64}(1.0,1.0,1.0),
    useMaterialColor::Bool=false, smoothNormals::Bool=false, convexPartition::Bool=false)

Generate a new solid or visual shape representing a mesh.
The reference frame = Object3D frame is defined by the mesh data.

# Arguments
- `filename` defines the name of the mesh file.
- `scale` defines the scaling factors to be applied in x-, y- and z-direction.
- `useMaterialColor` defines if the material color of the shape is to be considered.
- `smoothNormals` defines if smoothing of mesh normals is active.
- `convexPartition` defines if partitioning into convex sub meshes is active.

# Notes
- [Solid](@ref) features are supported only for [.obj files](https://en.wikipedia.org/wiki/Wavefront_.obj_file).
- [Visual](@ref) features are supported for (3ds, dxf, obj, stl) - formats.
- FileMesh is [not directly supported by animation export](https://github.com/ModiaSim/PrivateModia3D.jl/issues/77).
  You have to convert your .obj - FileMesh into a .json format first and store it in the same folder as your obj file.
"""
mutable struct FileMesh <: Modia3D.AbstractGeometry
    filename::AbstractString
    scaleFactor::SVector{3,Float64}
    useMaterialColor::Bool
    smoothNormals::Bool
    convexPartition::Bool

    # for solids only
    centroid::SVector{3,Float64}
    longestEdge::Float64
    objPoints::Vector{SVector{3,Float64}}
    facesIndizes::Vector{SVector{3,Int64}}
    volume::Float64
    centroidAlgo::SVector{3,Float64}
    inertia::SMatrix{3,3,Float64,9}
    function FileMesh(; filename::AbstractString="", scale=SVector{3,Float64}(1.0,1.0,1.0),
        useMaterialColor::Bool=false, smoothNormals::Bool=false, convexPartition::Bool=false)
        if !isfile(filename)
        error("FileMesh(\"$filename\",...): file not found.")
        end
        @assert(scale[1] >= 0.0)
        @assert(scale[2] >= 0.0)
        @assert(scale[3] >= 0.0)
        fileMesh = new(filename, scale, useMaterialColor, smoothNormals, convexPartition)
        return fileMesh
    end
end
