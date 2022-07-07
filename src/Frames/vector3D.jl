# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Frames (Modia3D/Frames/_module.jl)
#

"""
    Modia3D.ZeroVector3D(::Type{F}) where F = SVector{3,F}(0, 0, 0)

Constant of a SVector{3,Float64} where all elements are zero
"""
ZeroVector3D(::Type{F}) where F <: Modia3D.VarFloatType = SVector{3,F}(0, 0, 0)



"""
    vec = Modia3D.axisValue(axis, value::F) where F
    vec = Modia3D.axisValue(axis, positive, value::F) where F

Return `vec::SVector{3,F}` where all elements are zero with exception of `vec[axis] = value` or
`vec[axis] = positive ? value : -value`.
"""
@inline axisValue(axis::Int, value::F) where F =
                    axis==1 ? SVector{3,F}(value, 0, 0) :
                   (axis==2 ? SVector{3,F}(0, value, 0) :
                   (axis==3 ? SVector{3,F}(0, 0, value) :
                   error("Error when calling Modia3D.axisValue($axis, ...) - first argument needs to be 1,2 or 3.")))
@inline axisValue(axis::Int, positive::Bool, value::F) where F = positive ? axisValue(axis,value) : axisValue(axis,-value)




"""
    M = Modia3D.skew(e::SVector{3,F}) where F
    M = Modia3D.skew(e::Vector{F})    where F

Return the skew symmetric matrix `M` of vector `e` (`length(e) = 3`) with

```julia
M = @SMatrix [  F(0.0)  -e[3]    e[2];
                e[3]    F(0.0)  -e[1];
               -e[2]     e[1]   F(0.0) ]
```
"""
skew(e::SVector{3,F}) where F = @SMatrix [  F(0.0)  -e[3]    e[2];
                                            e[3]    F(0.0)  -e[1];
                                           -e[2]     e[1]   F(0.0) ]
skew(e::Vector{F})    where F = @SMatrix [  F(0.0)  -e[3]    e[2];
                                            e[3]    F(0.0)  -e[1];
                                           -e[2]     e[1]   F(0.0) ]
