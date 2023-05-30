# License for this file: MIT (expat)
# Copyright 2017-2022, DLR Institute of System Dynamics and Control

"""
    module Modia3D.Frames

This module contains functions for **frames** that is coordinate systems in 3D.
The orientation of a frame is described either with a 3x3 **rotation matrix**
or with a **quaternion vector** and its origin is described with a **SVector{3,Float64}**.
For details see Modia3D/docs/Functions.md.

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

export  rot1,  rot2,  rot3,  rot123, rotAxis,  rot_e,  rot_nxy,  rot_nxz, from_q
export qrot1, qrot2, qrot3, qrot123,          qrot_e, qrot_nxy, qrot_nxz, from_R

export resolve1, resolve2, absoluteRotation, relativeRotation, inverseRotation
export planarRotationAngle, eAxis
export angularVelocityResolvedInParentStates!

export Path, t_pathEnd, interpolate, interpolate_r
export skew

import Modia3D
using StaticArrays
using LinearAlgebra


include("vector3D.jl")
include("rotationMatrix.jl")
include("quaternion.jl")
include("interpolation.jl")

end
