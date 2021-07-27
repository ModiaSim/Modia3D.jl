# Collision Handling

Collision handling with elastic response calculation is performed for convex shapes that are defined with a contact material or solid material. The elastic response calculation is currently solely based on the information about the largest penetration that is computed with an improved Minkowski Portal Refinement (MPR) algorithm. Collision response with an adaptive integration method works only reasonable, if the convex objects have point-contact.

Collision handling can be globally activated with keyword `enableContactDetection = true` set in the [Scene](@ref). Only [Solid](@ref) [Object3D](@ref)s can take place in collision situations.

The example in `"$(Modia3D.path)/test/Collision/BouncingSphere.jl"` defines a sphere that is bouncing on the ground. The essential statements are:

```
world = Object3D(feature = Scene(enableContactDetection = true)), # true is default value

bouncingSphere = Object3D(
            feature = Solid(shape         = Sphere(diameter = 0.5),
                            solidMaterial = "Steel", # for mass and force computation
                            collision     = true)),      # enable collision flag
ground = Object3D(parent = :world,
            feature = Solid(shape = Box((lengthX=1.5, lengthY=0.5, lengthZ=0.2)),
                             solidMaterial = "Steel", # for mass and force computation
                             collision = true))       # enable collision flag

```

Note:

- Only [Solid](@ref) Object3Ds where a `shape` is defined and `collision=true` is take place in collision handling

- Supported [Shapes](@ref) are: [Sphere](@ref), [Ellipsoid](@ref), [Box](@ref), [Cylinder](@ref), [Cone](@ref), [Capsule](@ref), [Beam](@ref), [FileMesh](@ref).
  - [FileMesh](@ref):
    - Only [.obj files](https://en.wikipedia.org/wiki/Wavefront_.obj_file) are supported.
    - MPR algorithm uses the convex hull of a concave geometry, or you have to partition it into convex sub meshes with e.g., [V-HACD](https://github.com/kmammou/v-hacd).
    
- Define in the [Solid](@ref) a `solidMaterial="xxx"` or a `contactMaterial="yyy"` (this defines for example YoungsModulus). The used names must be available in the `Modia3D/palettes/contactPairMaterials.json` where for various combinations of contact materials, additional data is provided (for example the coefficientOfRestitution). For more details about the contact material data, see [Solid material](@ref) and [Contact pair material](@ref). The details of the contact computation are sketched in [Contact Force Law](@ref).