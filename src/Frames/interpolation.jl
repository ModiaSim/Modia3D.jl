# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Frames (Modia3D/Frames/_module.jl)
#


"""
    path = Modia3D.Path(r::Vector{SVector{3,Float64}},
                        q::Vector{SVector{4,Float64}} = NullQuaternion(Float64);
                        v = ones(length(r)))

Return an instance of a new Path object. The Path object consists of n frames
defined by the position vectors of their origins (`r[i]` for frame `i`)
and optionally of their absolute rotation quaternions (`q[i]` for frame `i`)
describing the rotation from the world frame to the respective frame.

A path parameter `t` is defined in the following way on these frames:

- `t[1] = 0.0`.
- `t[i] = t[i-1] + pathLength_i/(v[i]+v[i-1])/2` if the origins of frames `i-1` and `i` do not coincide.
- `t[i] = t[i-1] + pathAngle_i/(v[i]+v[i-1])/2` if the origins of frames `i-1` and `i` do coincide.

Hereby `pathLength_i` is the distance between the origins of frames `i-1` and `i` in [m] and
`pathAngle_i` is the planar rotation angle between frames `i-1` and `i` in [rad].

If `v[i]` is the desired velocity or angular velocity at frame `i`, then path parameter
`t` is approximately the time to move along the path. The time instant `t_end` of the last frame can
be inquired with `Modia3D.t_pathEnd(path)`. For example, if a simulation shall be performed in such
a way that the simulation should start with the first frame and end at `stopTime` at the last frame,
then the path parameter should be selected as `t = time*t_end/stopTime`.

Given the actual path parameter, typically `0 <= t <= t_end` (if `t` is outside of this interval,
the frame at `t` is determined by extrapolation through the first two or the last two frames),
the corresponding frame is determined by linear interpolation in the following way:

```julia
(rt, qt) = interpolate(  path,t)
 rt      = interpolate_r(path,t)
```

where `rt` is the position vector to the origin of the frame at path parameter `t`
and `qt` is the absolute quaternion of the frame at path parameter `t`.

# Example

```julia
import Modia3D
using Unitful

r = [ SVector{3,F}(1,0,0),
      SVector{3,F}(0,1,0),
      SVector{3,F}(0,0,1) ]
q = [ Modia3D.NullQuaternion(F),
      Modia3D.qrot1(45u"°"),
      Modia3D.qrot2(60u"°")]

path     = Modia3D.Path(r,q)
t_end    = Modia3D.t_pathEnd(path)
dt       = 0.1
stopTime = 2.0
time     = 0.0

while time <= stopTime
   (rt, qt) = Modia3D.interpolate(path, time*t_end/stopTime)
   time += dt
end
```
"""
struct Path
    t::Vector{Float64}        # path parameter; t=0 is (r[1],q[1])
    r::Vector{SVector{3,Float64}}       # Position vectors from world frame to origin of Frames
    q::Vector{SVector{4,Float64}}       # Quaternions describing rotation from world frame to Frames

    function Path(r::AbstractVector, q::AbstractVector=SVector{4,Float64}[];
                  v::AbstractVector=ones(length(r)),
                  seps=Modia3D.neps )
        nframes = size(r, 1)
        @assert(seps > 0.0)
        @assert(nframes > 1)
        @assert(length(v) == nframes)
        @assert(size(q, 1) == 0 || size(q, 1) == nframes)
        @assert(v[1]   >= 0.0)
        @assert(v[end] >= 0.0)
        for i in 2:length(v) - 1
            @assert(v[i] > 0.0)
        end
        vv = convert(Vector{Float64}, v)

        # Determine path length for every segment
        t = zeros(nframes)
        for i in 2:nframes
            slen = norm(r[i] - r[i - 1])
            if slen > seps
                # Use distance between r[i] and r[i-1] as path parameter (in [m]), scale with vv
                t[i] = t[i - 1] + slen / ((vv[i] + vv[i - 1]) / 2)

            elseif length(q) > 0
                # Use planar rotation angle between q[i] and q[i-1] as path parameter (in [rad])
                q_rel = relativeRotation(q[i - 1], q[i])
                q4    =  q_rel[4]
                q4    = q4 >  1 + seps ?  1.0 :
                    q4 < -1 - seps ? -1.0 : q4
                absAngle = 2 * acos(q4)

                if absAngle < seps
                    error("Modia3D.Path(..): r[i] == r[i-1] and q[i] == q[i-1] (i = ", i, ").")
                end
                t[i] = t[i - 1] + absAngle / ((vv[i] + vv[i - 1]) / 2)

            else
                error("Modia3D.Path(..): r[i] == r[i-1] (i = ", i, ").")
            end
        end
        new(t, r, q)
    end
end


"""
    t_end = Modia3D.t_pathEnd(path::[`Modia3D.Path`](@ref))

Return the final path parameter `t`of the last frame in path
(path parameter of first frame = 0.0).
"""
t_pathEnd(path::Path)::Float64 = path.t[end]



function get_interval(path::Path, t::Number)
    # returns index i, such that path.t[i] <= t < path.t[i+1]

    tvec = path.t
    if t <= 0.0
        return 1
    elseif t >= tvec[end]
        return length(tvec) - 1
    end

    low  = 1
    high = length(tvec)
    mid  = round(Int, (low + high) / 2)

    while t < tvec[mid] || t >= tvec[mid + 1]
        if t < tvec[mid]
            high = mid
        else
            low = mid
        end
        mid = round(Int, (low + high) / 2, RoundDown)
    end

    return mid
end



"""
    (rt, qt) = Modia3D.interpolate(path, t)

Return position `rt`and Quaternion `qt` of `path::`[`Modia3D.Path`](@ref) at path parameter `t::Number`.
"""
function interpolate(path::Path, t::Number)
    i    = get_interval(path, t)
    tvec = path.t
    tt::Float64  = convert(Float64, t)
    fac::Float64 = (tt - tvec[i]) / (tvec[i + 1] - tvec[i])
    rt = path.r[i] + fac * (path.r[i + 1] - path.r[i])
    qt = length(path.q) > 0 ? normalize(path.q[i] + fac * (path.q[i + 1] - path.q[i])) : NullQuaternion(Float64)

    return (rt, qt)
end



"""
    rt = Modia3D.interpolate_r(path, t)

Return position `r` of `path::`[`Modia3D.Path`](@ref) at path parameter `t::Number`.
"""
function interpolate_r(path::Path, t::Number)::SVector{3,Float64}
    i    = get_interval(path, t)
    tvec = path.t
    tt::Float64  = convert(Float64, t)
    fac::Float64 = (tt - tvec[i]) / (tvec[i + 1] - tvec[i])
    return path.r[i] + fac * (path.r[i + 1] - path.r[i])
end
