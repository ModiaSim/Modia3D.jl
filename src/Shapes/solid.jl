"""
    Solid(; shape           = nothing,
            solidMaterial   = nothing,
            massProperties  = nothing,
            visualMaterial  = VisualMaterial(),
            collision       = false,
            contactMaterial = "",
            collisionSmoothingRadius = 0.001,
            contactSphereRadius      = nothing)

Generate a [Solid](@ref) with physical behavior of a rigid body with mass, visualization and collision properties.
`Solid` is used as `feature` of an [Object3D](@ref).

# Arguments
- `shape`: Defines the shape of the solid. Only [Shapes](@ref) from [---- For Solid and Visual ----](@ref) can be used.
    If undefined the solid is not considered in visualization or [Collision Handling](@ref) and `massProperties` must
    be explicitly defined.

- `solidMaterial`: Defines the material of the solid used for mass properties computation and (if `contactMaterial` is
    undefined) contact force calculation. A pre-defined [Solid material](@ref) from palettes/solidMaterials.json (e.g.
    `"Steel"`) or a user-defined [Solid material](@ref) (e.g. `SolidMaterial(density=7850)`) can be used.
    `solidMaterial = ""` is treated as `solidMaterial = nothing`.

- `massProperties`: Defines the mass properties of the solid.
  - `massProperties = nothing`: The mass properties are computed from `shape` and `solidMaterial`.
    If additionally `solidMaterial = nothing`, the solid is treated as massless.
  - `massProperties = MassProperties(; mass=0.0, centerOfMass=[0.0,0.0,0.0], Ixx=0.0, Iyy=0.0, Izz=0.0)
    Ixy=4.5, Ixz=4.6, Iyz=5.5)`: Defines all mass properties explicitly
    `MassProperties()` defines a massless solid.
  - `massProperties = MassPropertiesFromShapeAndMass(; mass=0.0)`:
    The center of mass and the inertia tensor is computed from `shape` and from `mass`.

- `visualMaterial`: Defines the material of the solid used for visualization. A pre-defined [Visual material](@ref)
    from palettes/visualMaterials.json (e.g. `visualMaterial = "RedTransparent"`) or a user-defined [Visual material](@ref) (e.g.
    `VisualMaterial(color="DeepSkyBlue4", transparency=0.75)`) can be used.
    Note, `VisualMaterial = ""` is treated as `visualMaterial = VisualMaterial()`, so using the default visual material.

- `collision`: Defines if the solid is considered in [Collision Handling](@ref). `collision=true` requires definition
    of `shape` and of contact material (defined by `solidMaterial` or `contactMaterial`).

- `contactMaterial`: Defines the contact material in case of collision. For details, see [Contact Force Law](@ref) (e.g. `"Steel"`).
    If undefined `solidMaterial` is used as contact material. Only contact material combinations defined in
    palettes/contactPairMaterials.json can be used.

- `collisionSmoothingRadius`: Defines a collision smoothing radius for surface edges, its default value is `0.001` [m]. This improves the robustness of the integration, if contact is on or close to a surface edge. It takes the minimum value of the provided collision smoothing radius and 10% of the smallest shape length, like `min(collisionSmoothingRadius, 0.1 min(shape dimensions))`. If it is set to `0.0` no `collisionSmoothingRadius` is used. A `collisionSmoothingRadius` is used for `Box`, `Cylinder`, `Cone`, and `Beam`.

- `contactSphereRadius`: For each shape a `contactSphereRadius` is defined, so that a rough approximiation of Hertz' pressure is used in [Response calculation](@ref) not only for spheres. You can define your own `contactSphereRadius`, otherwise it is computed from shape geometry (sketched in the following table).

| Shape                     | `contactSphereRadius` from shape |
|:--------------------------|:---------------------------------|
| [Sphere](@ref)            | `diameter/2` |
| [Ellipsoid](@ref)         | `min(lengthX, lengthY, lengthZ)/2` |
| [Box](@ref)               | `min(lengthX, lengthY, lengthZ)/2` |
| [Cylinder](@ref)          | `min(diameter, length)/2` |
| [Cone](@ref)              | `(diameter + topDiameter)/4` |
| [Capsule](@ref)           | `diameter/2` |
| [Beam](@ref)              | `min(length, width, thickness)/2` |
| [FileMesh](@ref)          | `shortestEdge/2` |

For flat shapes, [Box](@ref) and [Beam](@ref), no `contactSphereRadius` is taken.  For Herz' pressure it is needed only if two flat shapes are colliding. Note, `contactSphereRadius <= 0` is treated as `contactSphereRadius = nothing`, so is computed from the table above.

# Example
```julia
using Modia3D

Solid(; shape = Sphere(diameter=0.5),
        solidMaterial = "DryWood",
        collision = true,
        contactMaterial = "Steel",
        collisionSmoothingRadius = 0.001,
        visualMaterial = VisualMaterial(color="DarkViolet"))
```
"""
struct Solid{F <: Modia3D.VarFloatType} <: Modia3D.AbstractObject3DFeature
    shape::Union{Modia3D.AbstractGeometry,Nothing}
    solidMaterial::Union{SolidMaterial,Nothing}
    massProperties::Union{MassProperties,Nothing}
    collision::Bool
    contactMaterial::Union{String,Modia3D.AbstractContactMaterial,Nothing}
    collisionSmoothingRadius::F
    visualMaterial::Union{Shapes.VisualMaterial,Nothing}
    isFlat::Bool
    contactSphereRadius::F

    function Solid{F}(;
        shape::Union{Modia3D.AbstractGeometry,Nothing} = nothing,
        solidMaterial::Union{Modia3D.AbstractMassPropertiesInterface,AbstractString,SolidMaterial,Nothing} = nothing,
        massProperties::Union{Modia3D.AbstractMassProperties, Number, AbstractString, SolidMaterial, Nothing} = nothing,
        collision::Bool = false,
        contactMaterial::AbstractString = "",
        collisionSmoothingRadius=F(0.001),
        visualMaterial::Union{Shapes.VisualMaterial,AbstractString,Nothing} = Shapes.VisualMaterial(),
        visualMaterialConvexDecomposition::Union{Shapes.VisualMaterial,AbstractString,Nothing} = Shapes.VisualMaterial(),
        contactSphereRadius::Union{Nothing, Number} = nothing) where F <: Modia3D.VarFloatType

        if collision && isnothing(shape)
            error("For collision/gripping simulations, a shape must be defined.")
        end

        if collision
            # it is a solid material
            if typeof(solidMaterial) == String && contactMaterial == ""
                contactMaterial = solidMaterial
            elseif typeof(solidMaterial) != String && contactMaterial == ""
                error("For collision handling an Object3Ds solidMaterial must be defined as a string or a contactMaterial must be set.")
            end
        else # no collision, therefore contactMaterial is ""
            contactMaterial = ""
        end

        if typeof(solidMaterial) == String && solidMaterial == ""
            solidMaterial = nothing
        end

        if typeof(solidMaterial) == String
            solidMaterial = solidMaterialPalette[1][solidMaterial]
        end

        if typeof(visualMaterial) == String
            visualMaterial = visualMaterial == "" ? Shapes.VisualMaterial() : Shapes.visualMaterialPalette[visualMaterial]
        end

        if typeof(shape) == FileMesh
            (shape.centroid, shape.shortestEdge, shape.longestEdge, shape.objPoints, shape.facesIndizes) = getMeshInfos(shape.filename, shape.scaleFactor)
            (shape.volume, shape.centroidAlgo, shape.inertia) = computeMassProperties(shape.objPoints, shape.facesIndizes; bodyCoords=false)
        end

        massProperties = isnothing(massProperties) && isnothing(solidMaterial) ? MassProperties{F}() : createMassProperties(F, massProperties, shape, solidMaterial)

        contactSphereRadius::Union{Nothing,F} = isnothing(contactSphereRadius) || F(contactSphereRadius) <= F(0) ? nothing : F(contactSphereRadius)
        (isFlat, contactSphereRadius) = setContactSphereRadius(shape, contactSphereRadius, F)
        new(shape, solidMaterial, massProperties, collision, contactMaterial, setCollisionSmoothingRadius(shape, F(collisionSmoothingRadius)), visualMaterial, isFlat, contactSphereRadius)
    end
end
Solid(args...; kwargs...) = Solid{Float64}(args...; kwargs...)


function JSON.show_json(io::JSON.StructuralContext, s::JSON.CommonSerialization, solid::Solid)
   JSON.begin_object(io)
      JSON.show_pair(io, s, "shape"          , solid.shape)
      JSON.show_pair(io, s, "massProperties" , solid.massProperties)
      JSON.show_pair(io, s, "material"       , solid.visualMaterial)
      JSON.show_pair(io, s, "contactMaterial", solid.contactMaterial)
   JSON.end_object(io)
end
