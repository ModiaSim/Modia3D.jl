# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Frames (Modia3D/Frames/_module.jl)
#

@eval using LinearAlgebra


"""
Constant rotation matrix that defines no rotation from frame 1 to frame 2.
"""
NullRotation(::Type{F}) where F <: AbstractFloat = SMatrix{3,3,F,9}(Matrix(F(1.0)I, 3, 3))



"""
    Modia3D.assertRotationMatrix(R::AbstractMatrix)

Assert that matrix R has the properties of a rotation matrix
(is 3x3 and R'*R - eye(3) = zeros(3,3))
"""
function assertRotationMatrix(R::AbstractMatrix)
   @assert(size(R,1)==3, size(R,2)==3)
   @assert(norm(R'*R - NullRotation(Float64)) <= 1e-10)
end


"""
    R = Modia3D.rot1(angle)

Return rotation matrix R that rotates with angle `angle` along the x-axis of frame 1.
"""
@inline function rot1(angle::F) where F <: AbstractFloat
   (s,c) = sincos(angle)
   R = SMatrix{3,3,F,9}([1.0  0.0  0.0;
                 0.0   c    s ;
                 0.0  -s    c ])
end


"""
    R = Modia3D.rot2(angle)

Return rotation matrix R that rotates with angle `angle` in [radian] along the y-axis of frame 1.
"""
@inline function rot2(angle::F) where F <: AbstractFloat
   (s,c) = sincos(angle)
   R = SMatrix{3,3,F,9}([ c  0.0 -s ;
                 0.0 1.0 0.0;
                  s  0.0  c ])
end


"""
    R = Modia3D.rot3(angle)

Return rotation matrix R that rotates with angle `angle` in [radian] along the z-axis of frame 1.
"""
@inline function rot3(angle::F) where F <: AbstractFloat
   (s,c) = sincos(angle)
   R = SMatrix{3,3,F,9}([ c   s  0.0;
                        -s    c  0.0;
                         0.0 0.0 1.0])
end


"""
    R = Modia3D.rot123(angle1, angle2, angle3)

Return rotation matrix R by rotating with angle1 along the x-axis of frame 1,
then with angle2 along the y-axis of this frame and then with angle3 along
the z-axis of this frame.
"""
rot123(angle1::F, angle2::F, angle3::F) where F <: AbstractFloat = rot3(angle3)*rot2(angle2)*rot1(angle1)


"""
    R = Modia3D.rotAxis(axis, angle)
    R = Modia3D.rotAxis(axis, positive, angle)

Return rotation matrix R that rotates with angle `angle` in [radian] along axis `axis` (= 1, 2 or 3), or
with `angle` if `positive=true` and otherwise with `-angle`.
"""
@inline rotAxis(axis::Int, angle::F) where F = axis==3 ? rot3(angle) : (axis==2 ? rot2(angle) :
        (axis==1 ? rot1(angle) : error("Bug in Modia3D: rotAxis($axis, ...) - argument needs to be 1,2 or 3.")))
@inline rotAxis(axis::Int, positive::Bool, angle::F) where F = positive ? rotAxis(axis, angle) : rotAxis(axis, -angle)


"""
    R = Modia3D.rot_e(e, angle)

Return rotation matrix that rotates around angle `angle` along unit axis `e`.
This function assumes that `norm(e) == 1`.
"""
@inline function rot_e(e::SVector{3,F},angle::F) where F
    (s,c) = sincos(angle)
    return SMatrix{3,3,F,9}(e*e' + (NullRotation(F) - e*e')*c - skew(e)*s)
end
rot_e(e::AbstractVector, angle::F) where F = rot_e( SVector{3,F}(e), F(angle) )


"""
    R = Modia3D.rot_nxy(nx, ny)

It is assumed that the two input vectors `nx` and `ny` are resolved in frame 1 and
are directed along the x and y axis of frame 2.
The function returns the rotation matrix R to rotate from frame 1 to frame 2.

The function is robust in the sense that it returns always a rotation matrix R,
even if `ny` is not orthogonal to `nx` or if one or both vectors have zero length.
This is performed in the following way:
If `nx` and `ny` are not orthogonal to each other, first a unit vector `ey` is
determined that is orthogonal to `nx` and is lying in the plane spanned by
`nx` and `ny`. If `nx` and `ny` are parallel or nearly parallel to each other
or `ny` is a vector with zero or nearly zero length, a vector `ey` is selected
arbitrarily such that `ex` and `ey` are orthogonal to each other.
If both `nx` and `ny` are vectors with zero or nearly zero length, an
arbitrary rotation matrix is returned.

# Example

```julia
using Unitful
import Modia3D

R1 = Modia3D.rot1(90u"°")
R2 = Modia3D.rot_nxy([1  , 0, 0], [0  , 0, 1  ])
R3 = Modia3D.rot_nxy([0.9, 0, 0], [1.1, 0, 1.1])
isapprox(R1,R2)   # returns true
isapprox(R1,R3)   # returns true
```
"""
function rot_nxy(nx::SVector{3,F}, ny::SVector{3,F}) where F <: AbstractFloat
  abs_nx  = norm(nx)
  e1      = abs_nx < 1e-10 ?  SVector{3,F}(1.0, 0.0, 0.0) : nx/abs_nx
  n3_aux  = cross(e1, ny)
  e2_aux  = dot(n3_aux,n3_aux) > 1e-6 ? ny : ( abs(e1[1]) > 1e-6 ? SVector{3,F}(0.0,1.0,0.0)
                                                                 : SVector{3,F}(1.0,0.0,0.0))
  n3_aux2 = cross(e1, e2_aux)
  e3      = normalize(n3_aux2)
  R       = SMatrix{3,3,F,9}(vcat(e1', cross(e3,e1)', e3'))
end
rot_nxy(nx::AbstractVector, ny::AbstractVector) = rot_nxy(SVector{3,F}(nx), SVector{3,F}(ny))



"""
    v1 = Modia3D.resolve1([R|q], v2)

Transform vector v2 (v resolved in frame 2) to vector v1 (v resolved in frame 1)
given either SMatrix ` R` or
quaternion `q` (to rotate a frame 1 into a frame 2).
"""
resolve1(R::SMatrix{3,3,F,9}, v2::SVector{3,F}) where F <: AbstractFloat = SVector{3,F}(R'*v2)
resolve1(R::SMatrix{3,3,F,9}, v2::AbstractVector) where F <: AbstractFloat = SVector{3,F}(R'*SVector{3,F}(v2))



"""
    v2 = Modia3D.resolve2([R|q], v1)

Transform vector v1 (v resolved in frame 1) to vector v2 (v resolved in frame 2)
given either SMatrix ` R` or
quaternion `q` (to rotate a frame 1 into a frame 2).
"""
resolve2(R::SMatrix{3,3,F,9}, v1::SVector{3,F}) where F <: AbstractFloat = SVector{3,F}(R*v1)
resolve2(R::SMatrix{3,3,F,9}, v1::AbstractVector) where F <: AbstractFloat = SVector{3,F}(R*SVector{3,F}(v1))


"""
     R2 = Modia3D.absoluteRotation(R1, R_rel)
     q2 = Modia3D.absoluteRotation(q1, q_rel)

Return rotation matrix `R2` or quaternion `q2`
defining the rotation from frame 0 to frame 2 from rotation matrix `R1` or quaternion `q1`that define the
rotation from frame 0 to frame 1 and the relative rotation matrix `R_rel` or the
relative quaternion `q_rel` that define the rotation from frame 1 to frame 2.
"""
absoluteRotation(R1::SMatrix{3,3,F,9}, R_rel::SMatrix{3,3,F,9}) where F <: AbstractFloat = SMatrix{3,3,F,9}(R_rel*R1)



"""
     R_rel = Modia3D.relativeRotation(R1, R2)
     q_rel = Modia3D.relativeRotation(q1, q2)

Return relative rotation matrix `R_rel` or relative
quaternion `q_rel` defining the rotation from frame 1 to frame 2
from absolute rotation matrix `R1` or absolute quaternion `q1` that define the
rotation from frame 0 to frame 1 and the absolute rotation matrix `R2` or the
absolute quaternion `q2` that define the rotation from frame 0 to frame 2.
"""
relativeRotation(R1::SMatrix{3,3,F,9}, R2::SMatrix{3,3,F,9}) where F <: AbstractFloat = SMatrix{3,3,F,9}(R2*R1')


"""
     R_inv = Modia3D.inverseRotation(R)
     q_inv = Modia3D.inverseRotation(q)

Return inverse rotation matrix `R_inv` or inverse
quaternion `q_inv` defining the rotation from frame 1 to frame 0
from rotation matrix `R` or quaternion `q` that define the
rotation from frame 0 to frame 1.
"""
inverseRotation(R::SMatrix{3,3,F,9}) where F <: AbstractFloat = SMatrix{3,3,F,9}(R')


"""
    angle = planarRotationAngle(e, v1, v2; angle_guess = 0.0)

Return `angle` of a planar rotation, given the normalized axis of
rotation to rotate frame 1 around `e` into frame 2 (norm(e) == 1 required),
and the representations of a vector in frame 1 (`v1`) and frame 2 (`v2`).
Hereby, it is required that `v1` is not parallel to `e`.
The returned angle is in the range `-pi <= angle - angle_guess <= pi`
(from the infinite many solutions, the one is returned that is closest to `angle_guess`).

# Example

```julia
import Modia3D
using Unitful

angle1 = 45u"°"
e      = normalize([1.0, 1.0, 1.0])
R      = Modia3D.rot_e(e, angle1)

v1 = [1.0, 2.0, 3.0]
v2 = Modia3D.resolve2(R, v1)

angle2 = planarRotationAngle(e, v1, v2)
isapprox(angle1, angle2)
```
"""
@inline function planarRotationAngle(e::AbstractVector, v1::AbstractVector, v2::AbstractVector; angle_guess::Number=0.0)::Number
   angle1 = atan( dot(-cross(e,v1), v2), dot(v1,v2) - dot(e,v1)*dot(e,v2) )
   pi2    = 2*pi
   return angle1 + pi2*round(Int, (pi+angle_guess-angle1)/(pi2), RoundDown)
end


#=
Derivation of algorithm for planarRotationAngle:

Vector v is resolved in frame 1 and frame 2 according to:
   (1)  v2 = (e*e' + (NullRotation(Float64) - e*e')*cos(angle) - skew(e)*sin(angle))*v1
           = e*(e'*v1) + (v1 - e*(e'*v1))*cos(angle) - cross(e,v1)*sin(angle)

Equation (1) is multiplied with "v1'" resulting in (note: e'*e = 1)

   (2)  v1'*v2 = (v1'*e)*(e'*v2) + (v1'*v1 - (v1'*e)*(e'*v1))*cos(angle)

and therefore

   (3)  cos(angle) = ( v1'*v2 - (v1'*e)*(e'*v2)) / (v1'*v1 - (v1'*e)*(e'*v1))

Similarly, equation (1) is multiplied with cross(e,v1), that is a
a vector that is orthogonal to e and to v1:

   (4)  cross(e,v1)'*v2 = -cross(e,v1)'*cross(e,v1)*sin(angle)

and therefore:

   (5) sin(angle) = -cross(e,v1)'*v2/(cross(e,v1)'*cross(e,v1))

We have e'*e=1. Therefore

   (6) v1'*v1 - (e*v1)'*(e*v1) = |v1|^2 - (|v1|*cos(e,v1))^2

and

   (7) cross(e,v1)'*cross(e,v1) = (|v1|*sin(e,v1))^2
                                 = |v1|^2*(1 - cos(e,v1)^2)
                                 = |v1|^2 - (|v1|*cos(e,v1))^2

The denominators of (3) and (5) are identical, according to (6) and (7).
Furthermore, the denominators are always positive according to (7).
Therefore, in the equation "angle = atan2(sin(angle), cos(angle))" the
denominators of sin(angle) and cos(angle) can be removed, resulting in:

   (8) angle1 = atan2(-cross(e,v1)'*v2, v1'*v2 - (e'*v1)*(e'*v2));

This angle is in the range -pi <= angle1 <= pi. The returned angle should be
as close to angle_guess. If angle_guess = 0, angle1 is just returned. Otherwise:

       -pi < angle - angle_guess <= pi
       -pi < angle1 + 2*pi*N - angle_guess <= pi
       (-pi+angle_guess-angle1)/(2*pi) < N <= (pi+angle_guess-angle1)/(2*pi)
       -> N := round(Int, (pi+angle_guess-angle1)/(2*pi), RoundDown )

resulting in

    (9) angle = angle1 + 2*pi*round(Int, (pi+angle_guess-angle1)/(2*pi))
=#


"""
    e = eAxis(axis::Int)

Return unit vector `e` in direction of axis `axis` (`axis` = 1,2,3 or -1,-2-,3).


# Example

```julia
import Modia3D

e1 = ModiMath.eAxis(1)    # e1 = SVector{3,F}(1.0,  0.0, 0.0)
e2 = ModiMath.eAxis(-2)   # d2 = SVector{3,F}(0.0, -1.0, 0.0)
```
"""
eAxis(::Type{F}, axis::Int) where F <: AbstractFloat = axis ==  1 ? SVector{3,F}(  1.0,  0.0,  0.0) :
                   axis ==  2 ? SVector{3,F}(  0.0,  1.0,  0.0) :
                   axis ==  3 ? SVector{3,F}(  0.0,  0.0,  1.0) :
                   axis == -1 ? SVector{3,F}( -1.0,  0.0,  0.0) :
                   axis == -2 ? SVector{3,F}(  0.0, -1.0,  0.0) :
                   axis == -3 ? SVector{3,F}(  0.0,  0.0, -1.0) :
                   error("Modia3D.eAxis(axis): axis = ", axis, " but must be 1, 2, 3, -1, -2, or -3.")
