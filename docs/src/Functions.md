# Functions

```@meta
CurrentModule = Modia3D
```

## 3D Vectors

Functions to construct 3D vectors `v::SVector{3,F}`.

```@docs
ZeroVector3D
axisValue
```

## Rotation Matrices

Functions to construct rotation matrices `R::SMatrix{3,3,F,9}` rotating a 
coordinate-system/frame 1 into a coordinate-system/frame 2.

| Function                                         | Description                                        |
|:-------------------------------------------------|:---------------------------------------------------|
| [`Modia3D.NullRotation`](@ref)(::Type{F})        | No rotation from frame 1 to frame 2                |
| [`Modia3D.assertRotationMatrix`](@ref)(R)        | Assert that R is a rotation matrix                 |
| [`Modia3D.rot1`](@ref)(angle)                    | Rotate around `angle` along x-axis                 |
| [`Modia3D.rot2`](@ref)(angle)                    | Rotate around `angle` along y-axis                 |
| [`Modia3D.rot3`](@ref)(angle)                    | Rotate around `angle` along z-axis                 |
| [`Modia3D.rot123`](@ref)(angle1, angle2, angle3) | Rotate around angles along x,y,z-axes              |
| [`Modia3D.rot123`](@ref)(angles)                 | Rotate around angles along x,y,z-axes              |
| [`Modia3D.rotAxis`](@ref)(axis,angle)            | Rotate around `angle` along `axis` (= 1,2,3)       |
| [`Modia3D.rotAxis`](@ref)(axis,positive,angle)   | Rotate around `angle` if `positive`, else `-angle` |
| [`Modia3D.rot_e`](@ref)(e, angle)                | Rotate around `angle` along unit vector `e`        |
| [`Modia3D.rot_nxy`](@ref)(nx, ny)                | `nx`/`ny` are in x/y-direction of frame 2          |
| [`Modia3D.from_q`](@ref)(q)                      | Return rotation matrix from quaternion `q`         |


Examples

```julia
using Modia3D

# R1,R2,R3 are the same RotationMatrices
R1 = Modia3D.rot1(pi/2)
R2 = Modia3D.rot1(90u"°")
R3 = Modia3D.rot_e([1,0,0], pi/2)
```

```@docs
NullRotation
assertRotationMatrix
rot1
rot2
rot3
rot123
rotAxis
rot_e
rot_nxy
from_q
```

## Quaternions

Functions to construct quaternions `q::SVector{4,F}` rotating a 
coordinate-system/frame 1 into a coordinate-system/frame 2.

| Function                                          | Description                                |
|:--------------------------------------------------|:-------------------------------------------|
| [`Modia3D.NullQuaternion`](@ref)(::Type{F})       | No rotation from frame 1 to frame 2        |
| [`Modia3D.assertQuaternion`](@ref)(q)             | Assert that q is a quaternion              |
| [`Modia3D.qrot1`](@ref)(angle)                    | Rotate around `angle` along x-axis         |
| [`Modia3D.qrot2`](@ref)(angle)                    | Rotate around `angle` along y-axis         |
| [`Modia3D.qrot3`](@ref)(angle)                    | Rotate around `angle` along z-axis         |
| [`Modia3D.qrot123`](@ref)(angle1, angle2, angle3) | Rotate around angles along x,y,z-axes      |
| [`Modia3D.qrot_e`](@ref)(e, angle)                | Rotate around `angle` along unit vector `e`|
| [`Modia3D.qrot_nxy`](@ref)(nx, ny)                | `nx`/`ny` are in x/y-direction of frame 2  |
| [`Modia3D.from_R`](@ref)(R)                       | Return `q` from rotation matrix `R`        |


```@docs
NullQuaternion
assertQuaternion
qrot1
qrot2
qrot3
qrot123
qrot_e
qrot_nxy
from_R
```

## Frame Transformations

Functions to transform vectors, rotation matrices, quaternions between coordinate systems
and functions to determine properties from coordinate system transformations.

| Function                                         | Description                                   |
|:-------------------------------------------------|:----------------------------------------------|
| [`Modia3D.resolve1`](@ref)(rot, v2)              | Transform vector `v` from frame 2 to frame 1  |
| [`Modia3D.resolve2`](@ref)(rot, v1)              | Transform vector `v` from frame 1 to frame 2  |
| [`Modia3D.absoluteRotation`](@ref)(rot01, rot12) | Return rotation 0->2 from rot. 0->1 and 1->2  |
| [`Modia3D.relativeRotation`](@ref)(rot01, rot02) | Return rotation 1->2 from rot. 0->1 and 0->2  |
| [`Modia3D.inverseRotation`](@ref)(rot01)         | Return rotation 1->0 from rot, 0->1           |
| [`Modia3D.planarRotationAngle`](@ref)(e,v1,v2)   | Return angle of planar rotation along `e`     |
| [`Modia3D.eAxis`](@ref)(axis)                    | Return unit vector `e` in direction of `axis` |
| [`Modia3D.skew`](@ref)(v)                        | Return skew-symmetric matrix of vector v      |

```@docs
resolve1
resolve2
absoluteRotation
relativeRotation
inverseRotation
planarRotationAngle
eAxis
skew
```

## Frame Interpolations

Given a set of coordinate-systems/frames by a vector `r` of position vectors (to their origins) and
and an optional vector `q` of Quaternions (of their absolute orientations), then
the following functions interpolate linearly in these frames:

| Function                                        | Description                                    |
|:------------------------------------------------|:-----------------------------------------------|
| [`Modia3D.Path`](@ref)(r,q)                     | Return path defined by a vector of frames      |
| [`Modia3D.t_pathEnd`](@ref)(path)               | Return path parameter `t_end` of last frame    |
| [`Modia3D.interpolate`](@ref)(path,t)           | Return `(rt,qt)` of Path at path parameter `t` |
| [`Modia3D.interpolate_r`](@ref)(path,t)         | Return `rt` of Path at path parameter `t`      |


```@docs
Path
t_pathEnd
interpolate
interpolate_r
```
