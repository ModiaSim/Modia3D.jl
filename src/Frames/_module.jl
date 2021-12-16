# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

"""
    module Modia3D.Frames

This module contains functions for **frames** that is coordinate systems in 3D.
The orientation of a frame is described either with a 3x3 **rotation matrix**
or with a **quaternion vector** and its origin is described with a **SVector{3,Float64}**:

- Rotation matrix `SMatrix{3,3,Float64,9}`:
  Type of a Rotation matrix to rotate from a frame 1 into a frame 2.

- `Quaternion = SVector{4,Float64}`:
  Type of a Quaternion vector to rotate from a frame 1 into a frame 2.


The following constants are defined

- `Modia3D.NullRotation(F)`:
  rotation matrix with no rotation from a frame 1 into a frame 2.

- `Modia3D.NullQuaternion(F)`:
  Quaternion vector with no rotation from a frame 1 into a frame 2.

- `Modia3D.ZeroVector3D(F)`:
  SVector{3,Float64} with only zero elements.

If an angle is given as an argument to one of the functions below, it might be a
number (interpreted as having unit `rad`) or a number with a unit
(for example: `using Unitful; angle = 90u"°"`).


# Constructors for a RotationMatrix R

The following functions return a `SMatrix{3,3,F,9}` R`
to rotate a frame 1 into a frame 2.

| Function                                         | Description                                        |
|:-------------------------------------------------|:---------------------------------------------------|
| [`Modia3D.rot1`](@ref)(angle)                    | Rotate around `angle` along x-axis                 |
| [`Modia3D.rot2`](@ref)(angle)                    | Rotate around `angle` along y-axis                 |
| [`Modia3D.rot3`](@ref)(angle)                    | Rotate around `angle` along z-axis                 |
| [`Modia3D.rot123`](@ref)(angle1, angle2, angle3) | Rotate around angles along x,y,z-axes              |
| [`Modia3D.rotAxis`](@ref)(axis,angle)            | Rotate around `angle` along `axis` (= 1,2,3)       |
| [`Modia3D.rotAxis`](@ref)(axis,positive,angle)   | Rotate around `angle` if `positive`, else `-angle` |
| [`Modia3D.rot_e`](@ref)(e, angle)                | Rotate around `angle` along unit vector `e`        |
| [`Modia3D.rot_nxy`](@ref)(nx, ny)                | `nx`/`ny` are in x/y-direction of frame 2          |
| [`Modia3D.from_q`](@ref)(q)                      | Return `R` from Quaternion `q`                     |


# Constructors for a Quaternion q

The following functions return a `Quaternion SVector{4,F}` ` q`
to rotate a frame 1 into a frame 2.
Since `q` and `-q` define the same rotation the constructor functions have
a keyword argument `q_guess::SVector{4,F} = NullQuaternion(F)`.
From the two possible solutions `q` the one is returned that is closer to `q_guess`.

| Function                                          | Description                                |
|:--------------------------------------------------|:-------------------------------------------|
| [`Modia3D.qrot1`](@ref)(angle)                    | Rotate around `angle` along x-axis         |
| [`Modia3D.qrot2`](@ref)(angle)                    | Rotate around `angle` along y-axis         |
| [`Modia3D.qrot3`](@ref)(angle)                    | Rotate around `angle` along z-axis         |
| [`Modia3D.qrot123`](@ref)(angle1, angle2, angle3) | Rotate around angles along x,y,z-axes      |
| [`Modia3D.qrot_e`](@ref)(e, angle)                | Rotate around `angle` along unit vector `e`|
| [`Modia3D.qrot_nxy`](@ref)(nx, ny)                | `nx`/`ny` are in x/y-direction of frame 2  |
| [`Modia3D.from_R`](@ref)(R)                       | Return `q` from SMatrix{3,3,F,9} `R`         |


# Operations on Frames

The following functions provide operations on frames. The orientation of a frame is
defined with argument `Rq` meaning it can be either a
rotation matrix ` R` or a
quaternion ` q` (to rotate a frame 1 into a frame 2).

| Function                                       | Description                                   |
|:-----------------------------------------------|:----------------------------------------------|
| [`Modia3D.resolve1`](@ref)(Rq, v2)             | Transform vector `v` from frame 2 to frame 1  |
| [`Modia3D.resolve2`](@ref)(Rq, v1)             | Transform vector `v` from frame 1 to frame 2  |
| [`Modia3D.absoluteRotation`](@ref)(Rq01, Rq12) | Return rotation 0->2 from rot. 0->1 and 1->2  |
| [`Modia3D.relativeRotation`](@ref)(Rq01, Rq02) | Return rotation 1->2 from rot. 0->1 and 0->2  |
| [`Modia3D.inverseRotation`](@ref)(Rq01)        | Return rotation 1->0 from rot, 0->1           |
| [`Modia3D.planarRotationAngle`](@ref)(e,v1,v2) | Return angle of planar rotation along `e`     |
| [`Modia3D.eAxis`](@ref)(axis)                  | Return unit vector `e` in direction of `axis` |
| [`Modia3D.skew`](@ref)(v)                      | Return skew-symmetric matrix of vector v      |


# Interpolation of Frames

Given a set of frames by a vector `r` of position vectors to their origins and
and an optional vector `q` of Quaternions of their absolute orientations, then
the following functions interpolate linearly in these frames:

| Function                                        | Description                                    |
|:------------------------------------------------|:-----------------------------------------------|
| [`Modia3D.Path`](@ref)(r,q)                     | Return path defined by a vector of frames      |
| [`Modia3D.t_pathEnd`](@ref)(path)               | Return path parameter `t_end` of last frame    |
| [`Modia3D.interpolate`](@ref)(path,t)           | Return `(rt,qt)` of Path at path parameter `t` |
| [`Modia3D.interpolate_r`](@ref)(path,t)         | Return `rt` of Path at path parameter `t`      |


# Examples

```julia
using Modia3D
using Unitful

# R1,R2,R3 are the same RotationMatrices
R1 = Modia3D.rot1(pi/2)
R2 = Modia3D.rot1(90u"°")
R3 = Modia3D.rot_e([1,0,0], pi/2)
```

# Main developer

[Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/),
[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

*The functions of this module are mostly a mapping of some of the functions
of the Modelica Standard Library from Modelica
([Modelica.Mechanics.MultiBody.Frames](https://doc.modelica.org/help/Modelica_Mechanics_MultiBody_Frames.html#Modelica.Mechanics.MultiBody.Frames))
to Julia (taking advantage of Julia features
such as multiple dispatch and unit package Unitful)*.
"""
module Frames

export NullRotation  , assertRotationMatrix
export NullQuaternion, assertQuaternion
export ZeroVector3D  , axisValue

export  rot1,  rot2,  rot3,  rot123, rotAxis,  rot_e,  rot_nxy, from_q
export qrot1, qrot2, qrot3, qrot123,          qrot_e, qrot_nxy, from_R

export resolve1, resolve2, absoluteRotation, relativeRotation, inverseRotation
export planarRotationAngle, eAxis

export Path, t_pathEnd, interpolate, interpolate_r
export skew

using StaticArrays
using LinearAlgebra


include("vector3D.jl")
include("rotationMatrix.jl")
include("quaternion.jl")
include("interpolation.jl")

end
