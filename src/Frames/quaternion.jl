# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Frames (Modia3D/Frames/_module.jl)
#


"""
    const Modia3D.Quaternion = SVector{4,Float64}

Describes the rotation from a frame 1 into a frame 2 with a quaternion vector.
If `e` is the (normalized) axis of rotation to rotate frame 1 into frame 2
(either resolved in frame 1 or frame 2) and `angle`
is the rotation angle for this rotation then the quaternion vector
`q::Modia3D.Quaternions` is defined as:

```julia
q = [e*sin(angle/2),
       cos(angle/2]
```
"""
const Quaternion = SVector{4,Float64}


"""
    const Modia3D.NullQuaternion = Quaternion(0,0,0,1)

Constant Quaternion vector of a null rotation (= no rotation from frame 1 to frame 2)
"""
NullQuaternion = Quaternion(0.0, 0.0, 0.0, 1.0)


"""
    Modia3D.assertQuaternion(q::AbstractVector)

Assert that vector `q` has the properties of a `Quaternion` vector
(has 4 elements, `norm(q) = 1`)
"""
function assertQuaternion(q::AbstractVector)
    @assert(length(q) == 4)
    @assert(abs(norm(q) - 1.0) <= 1e-10)
end


"""
    R = Modia3D.from_q(q::Modia3D.Quaternion)

Return RotationMatrix `R` from Quaternion `q`.
"""
from_q(q::Quaternion) = RotationMatrix(2.0 * (q[1] * q[1] + q[4] * q[4]) - 1.0,
                                       2.0 * (q[2] * q[1] - q[3] * q[4]),
                                       2.0 * (q[3] * q[1] + q[2] * q[4]),
                                       2.0 * (q[1] * q[2] + q[3] * q[4]),
                                       2.0 * (q[2] * q[2] + q[4] * q[4]) - 1.0,
                                       2.0 * (q[3] * q[2] - q[1] * q[4]),
                                       2.0 * (q[1] * q[3] - q[2] * q[4]),
                                       2.0 * (q[2] * q[3] + q[1] * q[4]),
                                       2.0 * (q[3] * q[3] + q[4] * q[4]) - 1.0)



const p4limit = 0.1
const c4limit = 4.0 * p4limit * p4limit

"""
    q = Modia3D.from_R(R::Modia3D.RotationMatrix;
                       q_guess = NullQuaternion)

Return `Quaternion q` from `RotationMatrix R`.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
function from_R(R::RotationMatrix;
                q_guess::Quaternion=NullQuaternion)::Quaternion
    # Based on https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    tt::Float64 = 0.0
    if R[3,3] < 0
        if R[1,1] > R[2,2]
            tt = 1 + R[1,1] - R[2,2] - R[3,3]
            q = Quaternion(tt, R[1,2] + R[2,1], R[3,1] + R[1,3], R[2,3] - R[3,2])
        else
            tt = 1 - R[1,1] + R[2,2] - R[3,3]
            q = Quaternion(R[1,2] + R[2,1], tt, R[2,3] + R[3,2], R[3,1] - R[1,3])
        end
    else
        if R[1,1] < -R[2,2]
            tt = 1 - R[1,1] - R[2,2] + R[3,3]
            q = Quaternion(R[3,1] + R[1,3], R[2,3] + R[3,2], tt, R[1,2] - R[2,1])
        else
            tt = 1 + R[1,1] + R[2,2] + R[3,3]
            q = Quaternion(R[2,3] - R[3,2], R[3,1] - R[1,3], R[1,2] - R[2,1], tt)
        end
    end
    q = q * 0.5/sqrt(tt)

    return dot(q, q_guess) >= 0 ? q : -q
end

from_R(R::AbstractMatrix; q_guess::AbstractVector=NullQuaternion)::Quaternion =
    from_R(RotationMatrix(R); q_guess=Quaternion(q_guess))



"""
    q = Modia3D.qrot1(angle; q_guess = NullQuaternion)

Return Quaternion `q` that rotates with angle `angle` along the x-axis of frame 1.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot1(angle::Number; q_guess::Quaternion=NullQuaternion)::Quaternion
    q = Quaternion(sin(angle / 2), 0.0, 0.0, cos(angle / 2))

    return dot(q, q_guess) >= 0 ? q : -q
end


"""
    q = Modia3D.qrot2(angle; q_guess = NullQuaternion)

Return Quaternion `q` that rotates with angle `angle` along the y-axis of frame 1.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot2(angle::Number; q_guess::Quaternion=NullQuaternion)::Quaternion
    q = Quaternion(0.0, sin(angle / 2), 0.0, cos(angle / 2))

    return dot(q, q_guess) >= 0 ? q : -q
end


"""
    q = Modia3D.qrot3(angle; q_guess = NullQuaternion)

Return Quaternion `q` that rotates with angle `angle` along the z-axis of frame 1.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot3(angle::Number; q_guess::Quaternion=NullQuaternion)::Quaternion
    q = Quaternion(0.0, 0.0, sin(angle / 2), cos(angle / 2))

    return dot(q, q_guess) >= 0 ? q : -q
end


absoluteRotation(q1::Quaternion, q_rel::Quaternion)::Quaternion =
     (@SMatrix [ q_rel[4]  q_rel[3] -q_rel[2] q_rel[1];
                -q_rel[3]  q_rel[4]  q_rel[1] q_rel[2];
                 q_rel[2] -q_rel[1]  q_rel[4] q_rel[3];
                -q_rel[1] -q_rel[2] -q_rel[3] q_rel[4]]) * q1


"""
    q = Modia3D.qrot123(angle1, angle2, angle3)

Return Quaternion `q` by rotating with angle1 along the x-axis of frame 1,
then with angle2 along the y-axis of this frame and then with angle3 along
the z-axis of this frame.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
qrot123(angle1::Number, angle2::Number, angle3::Number)::Quaternion = absoluteRotation(absoluteRotation(qrot1(angle1), qrot2(angle2)), qrot3(angle3))



"""
    q = Modia3D.qrot_e(e, angle; q_guess = NullQuaternion)

Return Quaternion `q` that rotates with angle `angle` along unit axis `e`.
This function assumes that `norm(e) == 1`.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot_e(e::SVector{3,Float64}, angle::Number; q_guess::Quaternion=NullQuaternion)::Quaternion
    sa = sin(angle / 2)
    q = Quaternion(e[1] * sa, e[2] * sa, e[3] * sa, cos(angle / 2))

    return dot(q, q_guess) >= 0 ? q : -q
end
qrot_e(e::AbstractVector, angle::Number)::Quaternion = qrot_e(SVector{3,Float64}(e), convert(Float64, angle))


"""
    q = Modia3D.qrot_nxy(nx, ny)

It is assumed that the two input vectors `nx` and `ny` are resolved in frame 1 and
are directed along the x and y axis of frame 2.
The function returns the Quaternion `q` to rotate from frame 1 to frame 2.

The function is robust in the sense that it returns always a Quaternion `q`,
even if `ny` is not orthogonal to `nx` or if one or both vectors have zero length.
This is performed in the following way:
If `nx` and `ny` are not orthogonal to each other, first a unit vector `ey` is
determined that is orthogonal to `nx` and is lying in the plane spanned by
`nx` and `ny`. If `nx` and `ny` are parallel or nearly parallel to each other
or `ny` is a vector with zero or nearly zero length, a vector `ey` is selected
arbitrarily such that `ex` and `ey` are orthogonal to each other.
If both `nx` and `ny` are vectors with zero or nearly zero length, an
arbitrary Quaternion `q` is returned.

# Example

```julia
using Unitful
import Modia3D

q1 = Modia3D.qrot1(90u"°")
q2 = Modia3D.qrot_nxy([1  , 0, 0], [0  , 0, 1  ])
q3 = Modia3D.qrot_nxy([0.9, 0, 0], [1.1, 0, 1.1])
isapprox(q1,q2)   # returns true
isapprox(q1,q3)   # returns true
```
"""
qrot_nxy(nx, ny)::Quaternion = from_R(rot_nxy(nx, ny))




resolve1(q::Quaternion, v2::SVector{3,Float64})::SVector{3,Float64} =
    2 * ((q[4] * q[4] - 0.5) * v2 + dot(q[1:3], v2) * q[1:3] + q[4] * cross(q[1:3], v2))

resolve2(q::Quaternion, v1::SVector{3,Float64})::SVector{3,Float64} =
    2 * ((q[4] * q[4] - 0.5) * v1 + dot(q[1:3], v1) * q[1:3] - q[4] * cross(q[1:3], v1))

relativeRotation(q1::Quaternion, q2::Quaternion)::Quaternion =
     (@SMatrix [ q1[4]  q1[3] -q1[2] -q1[1];
                -q1[3]  q1[4]  q1[1] -q1[2];
                 q1[2] -q1[1]  q1[4] -q1[3];
                 q1[1]  q1[2]  q1[3]  q1[4]]) * q2

inverseRotation(q::Quaternion)::Quaternion = Quaternion(-q[1], -q[2], -q[3], q[4])
