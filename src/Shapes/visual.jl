convertStringToVisualMaterial(visualMaterial::Shapes.VisualMaterial) = visualMaterial

convertStringToVisualMaterial(visualMaterial::AbstractString) = visualMaterial == "" ? Shapes.VisualMaterial() : Shapes.visualMaterialPalette[visualMaterial]


"""
    Visual(; shape          = nothing,
             visualMaterial = VisualMaterial())

Generate a new visual shape with a visual material. It is only for visualizing, without physical behavior.
The default is no shape and a default visual material.

# Arguments
- `shape`: It is possible to use all [Shapes](@ref) from both sets visual and solid, e.g. [`Sphere`](@ref), [`Box`](@ref), [`CoordinateSystem`](@ref). The default value is `nothing`, this means it will not be visualized.
- `visualMaterial`: Defines the material of the shape used for visualization. A pre-defined [Visual material](@ref)
    from palettes/visualMaterials.json (e.g. `visualMaterial = "RedTransparent"`) or a user-defined [Visual material](@ref) (e.g.
    `VisualMaterial(color="DeepSkyBlue4", transparency=0.75)`) can be used.
    Note, `VisualMaterial = ""` is treated as `visualMaterial = VisualMaterial()`, so using the default visual material.

# Examples
```julia
using Modia3D

shape1 = Visual(shape = Sphere(diameter = 0.5),
                visualMaterial = VisualMaterial(color = "Green"));
shape2 = Visual(shape = Sphere(diameter = 0.5),
                visualMaterial = "GreenTransparent");
```
"""
struct Visual <: Modia3D.AbstractObject3DFeature
    shape::Union{Modia3D.AbstractShape,Nothing}
    visualMaterial::Shapes.VisualMaterial

    function Visual(;
        shape::Union{Modia3D.AbstractShape,Nothing} = nothing,
        visualMaterial::Union{Shapes.VisualMaterial,AbstractString,Nothing} = Shapes.VisualMaterial())
        new(shape, convertStringToVisualMaterial(visualMaterial) )
    end
end
