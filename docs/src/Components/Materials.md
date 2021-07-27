# Materials

Modia3D uses various materials for different kind of purposes. In this section an overview
of the material data and material palettes is given, as well as links to more detailed
information. The following materia data is supported

- [Visual material](@ref): Visualization properties of a geometrical object,
  such as `color` or `transparency`.

- [Solid material](@ref): Material constants of one solid, such as
  `density` or `Young's modulus`.

- [Contact pair material](@ref): Material constants that are related to two solids that are in contact to each other, such as the coefficient of restitution between a Steel and an Aluminium object.


## Visual material

The mutable struct [`Modia3D.VisualMaterial`](@ref) defines
various visual properties of a geometrical object, such as *color* or *transparency*.
This data is used for visualization. Currently, there is no dictionary provided.

```@docs
Modia3D.Shapes.VisualMaterial
```


## Solid material

A [Solid](@ref) material is needed for computing mass properties with inertia (from `density` and the object's `shape`) and for [Collision Handling](@ref) (with `YoungsModulus` and `PoissonsRatio`).
There are predefined `solidMaterials` in `Modia3D/palettes/solidMaterials.json`. The material name (a string) is the key for reading and creating a [Solid material](@ref).

```@docs
Modia3D.Shapes.SolidMaterial
```

### Content of solid material palette
This is how a solid material is defined in `Modia3D/palettes/solidMaterials.json`.
```
"Steel": {
    "density": 8000.0,
    "YoungsModulus": 2.0e11,
    "PoissonsRatio": 0.3,
    "meltingPoint": 1640.0,
    "specificHeatCapacity": 500.0,
    "thermalConductivity": 50.0,
    "linearThermalExpansionCoefficient": 1.2e-5
}
```

List of all available keys in `Modia3D/palettes/solidMaterials.json`. You can add your own `solidMaterial` to the list.
```@repl
import Modia3D
Modia3D.listKeys(Modia3D.solidMaterialPalette)
```


## Contact pair material

The file `Modia3D/palettes/contactPairMaterials.json` provides
material constants that are related to two solids that are in contact to each other, for example the `coefficientOfRestitution` between a `"Steel"` and another `"Steel"` object.

```@docs
Modia3D.Shapes.ElasticContactPairMaterial
```

### Content of contact pairs palette
This is how a contact pair of two `"Steel"` pairing is defined in `Modia3D/palettes/contactPairMaterials.json`.
```
"Steel,Steel": {
    "responseType": "ElasticResponse",
    "coefficientOfRestitution": 0.7,
    "slidingFrictionCoefficient": 0.5,
    "rotationalResistanceCoefficient": 0.001
```
All available contact pairs. You can add your own contact pair material to this palette. It is filled during the first usage of Modia3D from file `Modia3D/palettes/contactPairMaterials.json`.

```@repl
import Modia3D
Modia3D.listKeys(Modia3D.contactPairMaterialPalette)
```
