convertStringToVisualMaterial(visualMaterial::Shapes.VisualMaterial) = visualMaterial

convertStringToVisualMaterial(visualMaterial::AbstractString) = Shapes.visualMaterialPalette[visualMaterial]

"""
    Visual(;
        shape = Sphere(diameter = 0.5),
        visualMaterial = VisualMaterial(color = "Green"))

Generate a new visual shape with a visual material. It is only for visualizing, without physical behavior. In this case it is a greenish sphere.

# Arguments
- `shape`: it is possible to use all [Shapes](@ref) from both sets visual and solid, e.g. [`Sphere`](@ref), [`Box`](@ref), [`CoordinateSystem`](@ref). The default value is `nothing`, this means it will not be visualized.
- `visualMaterial`: use a predefined [Visual material](@ref) defined in palettes/visualMaterials.json, or define it with `VisualMaterial(color = "Blue")`. The default value is `VisualMaterial()`.
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
