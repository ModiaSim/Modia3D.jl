# Dynamic Dispatch

Modia3D uses several abstract types in structs in order to store information
about different types of joints, shapes, collision response laws etc. Several functions
are used to operate on these types. A direct implementation would lead to
**dynamic dispatch**, so the functions to be called and the types to be operated
on, would be determined at run-time (and not at compile time). This leads to slow simulation
and a lot of memory allocation during simulation.

In this [forum discussion](https://groups.google.com/forum/#!msg/julia-users/jUMu9A3QKQQ/qjgVWr7vAwAJ) a similar issue is discussed for a tiny use case and three solutions are compared: (1) dynamic dispatch, (2) dictionary lookup, and (3) if-clauses. In this simple example version (3) is about a factor of **15 faster** as (1) and allocates 200 bytes, whereas (1) allocates 1.5 Mbyte.

In the rest of this section the technique is sketched that is used in Modia3D to avoid (a) dynamic dispatch and (b) access of abstract types *during simulation*:

The **Object3D** struct holds all information about an Object3D object:

```julia
@enum JointKind FixKind RevoluteKind PrismaticKind ...
@enum ShapeKind NoShapeKind SphereKind BoxKind CylinderKind ...

mutable struct Object3D
    ...
    # Relative motion parent/Object3D
    joint::AbstractJoint               # ::Fix, ::Revolute, ...

    # Efficient access of joint properties
    jointKind::JointKind               # Kind of joint
    ndof::Int                          # Number of degrees of freedom
    canCollide::Bool                   # = false, if no collision parent/Object3D

    # Feature associated with Object3D
    feature::AbstractObject3DFeature   # ::EmptyObject3DFeature ::Scene, ::Visual, ::Solid

    # Efficient access of feature properties
    shapeKind::Shapes.ShapeKind        # Kind of shape
    shape::Modia3D.AbstractShape
    visualMaterial::Shapes.VisualMaterial
    ...
end
```

In particular it holds *abstract types* for the `joint` and the `feature` associated with the `Object3D`. During initialization, the properties of `joint` and `feature` are copied to storage locations that have a *concrete type*. 

All operations on the abstract types are implemented with *if-clauses* and only access *concrete types*. For example, *Support points* needed to determine the shortest distances between colliding `Object3D`'s are computed with:

```julia
function supportPoint(obj::Object3D, e::SVector{3,Float64})::SVector{3,Float64}
    shapeKind                = obj.shapeKind   # type Int
    solid::Modia3D.Solid     = obj.feature
    collisionSmoothingRadius = solid.collisionSmoothingRadius  # type Float64

    if shapeKind == Modia3D.SphereKind
        return supportPoint_Sphere(obj.shape, obj.r_abs, obj.R_abs, e)
    elseif shapeKind == Modia3D.EllipsoidKind
        return supportPoint_Ellipsoid(obj.shape, obj.r_abs, obj.R_abs, e)
    elseif shapeKind == Modia3D.BoxKind
        return supportPoint_Box(obj.shape, obj.r_abs, obj.R_abs, e, collisionSmoothingRadius)
    ...
    end
end
```

Note, a function such as `supportPoint_Sphere(..)` requires that its first argument is an instance of the concrete type `Sphere`. Due to the if-clause, this is guaranteed. Julia will just make a cheap run-time check that this requirement is fulfilled. The concrete function (`supportPoint_Sphere`) is translated when llvm code is generated.
