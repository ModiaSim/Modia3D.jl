"""
    CoordinateSystem(; length=1.0)

Generate a new visual shape representing a coordinate system.
The reference frame = Object3D frame coincides with the coordinate system.

# Arguments
- `length` defines the length of the coordinate system axes.
"""
mutable struct CoordinateSystem <: Modia3D.AbstractShape
    length::Float64

    function CoordinateSystem(; length=1.0)
        @assert(length >= 0.0)
        new(length)
    end
end

"""
    Grid(; axis=3, length=1.0, width=0.6, distance=0.1, lineWidth=1.0)

Generate a new visual shape representing a planar grid.
The reference frame = Object3D frame is located at the center of the grid.

# Arguments
- `axis` defines the normal axis of the grid: 1=x; 2=y; 3=z.
- `length` defines the length of the grid.
- `width` defines the width of the grid.
- `distance` defines the distance of the grid lines.
- `lineWidth` defines the line width of the grid.

The dimension directions depend on `axis` by circularly shift:

| `axis` | `length` | `width` |
|:------:|:--------:|:-------:|
|   1    |   y      |   z     |
|   2    |   z      |   x     |
|   3    |   x      |   y     |

"""
mutable struct Grid <: Modia3D.AbstractShape
    axis::Int
    length::Float64
    width::Float64
    distance::Float64
    lineWidth::Float64

    function Grid(; axis=3, length=1.0, width=0.6, distance=0.1, lineWidth=1.0)
        @assert(1 <= axis <= 3)
        @assert(length >= 0.0)
        @assert(width >= 0.0)
        @assert(0.0 < distance <= min(length, width))
        @assert(lineWidth > 0.0)
        new(axis, length, width, distance, lineWidth)
    end
end

"""
    Spring(; axis=3, length=1.0, diameter=1.0, wireDiameter=0.05, windings=5)

Generate a new visual shape representing a helical spiral spring.
The reference frame = Object3D frame is located at the center of the base circle of the spring.

# Arguments
- `axis` defines the axis of the spring: 1=x; 2=y; 3=z.
- `length` defines the length of the spring.
- `diameter` defines the diameter of the spring.
- `wireDiameter` defines the diameter of the spring wire.
- `windings` defines the number of windings of the spring.

# Notes
- SimVis visualizes whole-numbered values of `windings`.
- Spring is [not supported by animation export](https://github.com/ModiaSim/PrivateModia3D.jl/issues/77).
"""
mutable struct Spring <: Modia3D.AbstractShape
    axis::Int
    length::Float64
    diameter::Float64
    wireDiameter::Float64
    windings::Float64

    function Spring(; axis=3, length=1.0, diameter=1.0, wireDiameter=0.05, windings=5)
        @assert(1 <= axis <= 3)
        @assert(length >= 0.0)
        @assert(diameter >= 0.0)
        @assert(0.0 < wireDiameter < min(diameter, length))
        new(axis, length, diameter, wireDiameter, windings)
    end
end


"""
    GearWheel(; axis=3, diameter=1.0, length=1.0, innerDiameter=0.0, angle=0.0, teeth=20)

Generate a new visual shape representing a gearwheel.
The reference frame = Object3D frame is located at the center of the gearwheel.

# Arguments
- `axis` defines the rotation axis of the gearwheel: 1=x; 2=y; 3=z.
- `diameter` defines the diameter of the gearwheel.
- `length` defines the length of the gearwheel.
- `innerDiameter` defines the inner diameter of the gearwheel (where `innerDiameter=0` defines a full gearwheel).
- `angle` defines the bevel angle of the gearwheel (where `angle=0` defines a cylindrical gearwheel).
- `teeth` defines the number of teeth of the gearwheel.

# Notes
- GearWheel is [not supported by animation export](https://github.com/ModiaSim/PrivateModia3D.jl/issues/77).
"""
mutable struct GearWheel <: Modia3D.AbstractShape
    axis::Int
    diameter::Float64
    length::Float64
    innerDiameter::Float64
    angle::Float64
    teeth::Int

    function GearWheel(; axis=3, diameter=1.0, length=1.0, innerDiameter=0.0, angle=0.0, teeth=20)
        @assert(1 <= axis <= 3)
        @assert(diameter >= 0.0)
        @assert(length >= 0.0)
        @assert(0.0 <= innerDiameter < diameter)
        @assert(isinteger(teeth))
        @assert(teeth > 0)
        new(axis, diameter, length, innerDiameter, angle, teeth)
    end
end


"""
    ModelicaShape(; type=1, lengthX=1.0, lengthY=1.0, lengthZ=1.0, extra=[0.0, 0.0, 0.0])

Generate a new visual shape according to the [Modelica](https://modelica.org) [Visualization Library](https://www.systemcontrolinnovationlab.de/the-dlr-visualization-library/).

# Arguments
- `type` defines the type of the shape: 1=box; 2=sphere; 3=cylinder; 4=cone; 5=capsule; 6=coordinate system;
  7=spring; 8=gearwheel; 9=pipe; 10=grid; 11=beam.
- `lengthX` defines the length of the shape in x-direction.
- `lengthY` defines the length of the shape in y-direction.
- `lengthZ` defines the length of the shape in z-direction.
- `extra` defines extra parameters for cone, pipe, spring and gearwheel.

# Notes
- ModelicaShape is intended for import of Modelica models and not for Modia3D models.
- ModelicaShape is [not supported by animation export](https://github.com/ModiaSim/PrivateModia3D.jl/issues/77).
"""
mutable struct ModelicaShape <: Modia3D.AbstractShape
    type::Int
    lengthX::Float64
    lengthY::Float64
    lengthZ::Float64
    extra::MVector{3,Float64}

    function ModelicaShape(; type=1, lengthX=1.0, lengthY=1.0, lengthZ=1.0, extra=[0.0, 0.0, 0.0])
        @assert(1 <= type <= 11)
        @assert(lengthX >= 0.0)
        @assert(lengthY >= 0.0)
        @assert(lengthZ >= 0.0)
        new(type, lengthX, lengthY, lengthZ, extra)
    end
end


getShapeKind(shape::Sphere)           = SphereKind
getShapeKind(shape::Ellipsoid)        = EllipsoidKind
getShapeKind(shape::Box)              = BoxKind
getShapeKind(shape::Cylinder)         = CylinderKind
getShapeKind(shape::Cone)             = ConeKind
getShapeKind(shape::Capsule)          = CapsuleKind
getShapeKind(shape::Beam)             = BeamKind
getShapeKind(shape::FileMesh)         = FileMeshKind
getShapeKind(shape::Spring)           = SpringKind
getShapeKind(shape::CoordinateSystem) = CoordinateSystemKind
getShapeKind(shape::Grid)             = GridKind
getShapeKind(shape::GearWheel)        = GearWheelKind
getShapeKind(shape::ModelicaShape)    = ModelicaKind
getShapeKind(shape::TextShape)        = TextKind
getShapeKind(shape)                   = UndefinedShapeKind
