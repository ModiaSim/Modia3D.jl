# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Frames (Modia3D/Frames/_module.jl)
#


"""
Describes the rotation from a frame 1 into a frame 2 with a SVector{4,F} vector.
If `e` is the (normalized) axis of rotation to rotate frame 1 into frame 2
(either resolved in frame 1 or frame 2) and `angle`
is the rotation angle for this rotation then the SVector{4,F} vector
`q::SVector{4,F}s` is defined as:

```julia
q = [e*sin(angle/2),
       cos(angle/2]
```
"""
const Quaternion = SVector{4,Float64}


"""
    const Modia3D.NullQuaternion(F) = SVector{4,F}(0.0, 0.0, 0.0, 1.0)

Constant SVector{4,F} vector of a null rotation (= no rotation from frame 1 to frame 2)
"""
NullQuaternion(::Type{F}) where {F} = SVector{4,F}(0.0, 0.0, 0.0, 1.0)



"""
    Modia3D.assertQuaternion(q::AbstractVector)

Assert that vector `q` has the properties of a `SVector{4,F}` vector
(has 4 elements, `norm(q) = 1`)
"""
function assertQuaternion(q::AbstractVector)
    @assert(length(q) == 4)
    @assert(abs(norm(q) - 1.0) <= 1e-10)
end


"""
    R = Modia3D.from_q(q::SVector{4,F})

Return SMatrix{4,4,F,16} `R` from SVector{4,F} `q`.
"""
from_q(q::SVector{4,F}) where {F} = SMatrix{3,3,F,9}( 2.0 * (q[1] * q[1] + q[4] * q[4]) - 1.0,
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
    q = Modia3D.from_R(R::SMatrix{3,3,F,9};
                       q_guess = NullQuaternion(F))

Return `SVector{4,F} q` from `SMatrix{3,3,F,9} R`.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
function from_R(R::SMatrix{3,3,F,9};
                q_guess::SVector{4,F}=NullQuaternion(F)) where {F}
    # Based on https://d3cw3dd2w32x2b.cloudfront.net/wp-content/uploads/2015/01/matrix-to-quat.pdf
    tt::F = F(0.0)
    if R[3,3] < 0
        if R[1,1] > R[2,2]
            tt = F(1) + R[1,1] - R[2,2] - R[3,3]
            q = SVector{4,F}(tt, R[1,2] + R[2,1], R[3,1] + R[1,3], R[2,3] - R[3,2])
        else
            tt = F(1) - R[1,1] + R[2,2] - R[3,3]
            q = SVector{4,F}(R[1,2] + R[2,1], tt, R[2,3] + R[3,2], R[3,1] - R[1,3])
        end
    else
        if R[1,1] < -R[2,2]
            tt = F(1) - R[1,1] - R[2,2] + R[3,3]
            q = SVector{4,F}(R[3,1] + R[1,3], R[2,3] + R[3,2], tt, R[1,2] - R[2,1])
        else
            tt = F(1) + R[1,1] + R[2,2] + R[3,3]
            q = SVector{4,F}(R[2,3] - R[3,2], R[3,1] - R[1,3], R[1,2] - R[2,1], tt)
        end
    end
    q = q * F(0.5)/sqrt(tt)

    return dot(q, q_guess) >= 0 ? SVector{4,F}(q) : SVector{4,F}(-q)
end

from_R(R::AbstractMatrix; q_guess::AbstractVector=NullQuaternion(F)) where {F}=
    from_R(SMatrix{3,3,F,9}(R); q_guess=SVector{4,F}(q_guess))



"""
    q = Modia3D.qrot1(angle; q_guess = NullQuaternion(F))

Return SVector{4,F} `q` that rotates with angle `angle` along the x-axis of frame 1.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot1(angle::F; q_guess::SVector{4,F}=NullQuaternion(F)) where {F}
    q = SVector{4,F}(sin(angle / 2), 0.0, 0.0, cos(angle / 2))
    return dot(q, q_guess) >= 0 ? SVector{4,F}(q) : SVector{4,F}(-q)
end


"""
    q = Modia3D.qrot2(angle; q_guess = NullQuaternion(F))

Return SVector{4,F} `q` that rotates with angle `angle` along the y-axis of frame 1.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot2(angle::F; q_guess::SVector{4,F}=NullQuaternion(F)) where {F}
    q = SVector{4,F}(0.0, sin(angle / 2), 0.0, cos(angle / 2))
    return dot(q, q_guess) >= 0 ? SVector{4,F}(q) : SVector{4,F}(-q)
end


"""
    q = Modia3D.qrot3(angle; q_guess = NullQuaternion(F))

Return SVector{4,F} `q` that rotates with angle `angle` along the z-axis of frame 1.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot3(angle::F; q_guess::SVector{4,F}=NullQuaternion(F)) where {F}
    q = SVector{4,F}(0.0, 0.0, sin(angle / 2), cos(angle / 2))
    return dot(q, q_guess) >= 0 ? SVector{4,F}(q) : SVector{4,F}(-q)
end


absoluteRotation(q1::SVector{4,F}, q_rel::SVector{4,F}) where {F} =
    SVector{4,F}(SMatrix{4,4,F,16}([ q_rel[4]  q_rel[3] -q_rel[2] q_rel[1];
                                   -q_rel[3]  q_rel[4]  q_rel[1] q_rel[2];
                                    q_rel[2] -q_rel[1]  q_rel[4] q_rel[3];
                                   -q_rel[1] -q_rel[2] -q_rel[3] q_rel[4]]) * q1)


"""
    q = Modia3D.qrot123(angle1, angle2, angle3)

Return SVector{4,F} `q` by rotating with angle1 along the x-axis of frame 1,
then with angle2 along the y-axis of this frame and then with angle3 along
the z-axis of this frame.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
qrot123(angle1::F, angle2::F, angle3::F) where {F} = absoluteRotation(absoluteRotation(qrot1(angle1), qrot2(angle2)), qrot3(angle3))



"""
    q = Modia3D.qrot_e(e, angle; q_guess = NullQuaternion(F))

Return SVector{4,F} `q` that rotates with angle `angle` along unit axis `e`.
This function assumes that `norm(e) == 1`.

From the two possible solutions `q` the one is returned that is closer
to `q_guess` (note, `q` and `-q` define the same rotation).
"""
@inline function qrot_e(e::SVector{3,F}, angle::F; q_guess::SVector{4,F}=NullQuaternion(F)) where {F}
    sa = sin(angle / 2)
    q = SVector{4,F}(e[1] * sa, e[2] * sa, e[3] * sa, cos(angle / 2))
    return dot(q, q_guess) >= 0 ? SVector{4,F}(q) : SVector{4,F}(-q)
end
qrot_e(e::AbstractVector, angle::F) where {F} = qrot_e(SVector{3,F}(e), F(angle))


"""
    q = Modia3D.qrot_nxy(nx, ny)

It is assumed that the two input vectors `nx` and `ny` are resolved in frame 1 and
are directed along the x and y axis of frame 2.
The function returns the SVector{4,F} `q` to rotate from frame 1 to frame 2.

The function is robust in the sense that it returns always a SVector{4,F} `q`,
even if `ny` is not orthogonal to `nx` or if one or both vectors have zero length.
This is performed in the following way:
If `nx` and `ny` are not orthogonal to each other, first a unit vector `ey` is
determined that is orthogonal to `nx` and is lying in the plane spanned by
`nx` and `ny`. If `nx` and `ny` are parallel or nearly parallel to each other
or `ny` is a vector with zero or nearly zero length, a vector `ey` is selected
arbitrarily such that `ex` and `ey` are orthogonal to each other.
If both `nx` and `ny` are vectors with zero or nearly zero length, an
arbitrary SVector{4,F} `q` is returned.

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
qrot_nxy(nx, ny)::SVector{4,F} = from_R(rot_nxy(nx, ny))



resolve1(q::SVector{4,F}, v2::SVector{3,F}) where {F} =
    SVector{3,F}(2 * ((q[4] * q[4] - F(0.5) ) * v2 + dot(q[1:3], v2) * q[1:3] + q[4] * cross(q[1:3], v2)) )

resolve2(q::SVector{4,F}, v1::SVector{3,F}) where {F} =
    SVector{3,F}(2 * ((q[4] * q[4] - F(0.5) ) * v1 + dot(q[1:3], v1) * q[1:3] - q[4] * cross(q[1:3], v1)) )

relativeRotation(q1::SVector{4,F}, q2::SVector{4,F}) where {F} =
    SVector{4,F}(SMatrix{4,4,F,16}( [ q1[4]  q1[3] -q1[2] -q1[1];
                                    -q1[3]  q1[4]  q1[1] -q1[2];
                                     q1[2] -q1[1]  q1[4] -q1[3];
                                     q1[1]  q1[2]  q1[3]  q1[4]]) * q2)

inverseRotation(q::SVector{4,F}) where {F} = SVector{4,F}(-q[1], -q[2], -q[3], q[4])
