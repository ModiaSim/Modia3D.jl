
# Materials

Modia3D uses various materials for different kind of purposes. In this section an overview
of the material data and material palettes is given, as well as links to more detailed
information. The following materia data is supported

- [Color material](@ref): Color names and associated RGB values.

- [Visual material](@ref): Visualization properties of a geometrical object,
  such as *color* or *transparency*.

- [Solid material](@ref): Material constants of *one solid*, such as
  *density* or *Young's modulus*.

- [Contact pair material](@ref): Material constants that are related to *two*
  solids that are in contact to each other, such as the *coefficient of restitution*
  between a Steel and an Aluminium object.


## Color material

 Color can be defined by a name, such as `Red`, and this
 name is associated with an Int-vector where the entries correspond to RGB values.
 Default color definitions are available via the [`Modia3D.colorPalette`](@ref)
 dictionary, where the color name is used as key and which is filled during the first
 usage of Modia3D from file `Modia3D/palettes/colors.json`.


### Content of colors palette

```@repl
import Modia3D
sort(Modia3D.colorPalette)
red_rgb = Modia3D.colorPalette["Red"]
```


## Visual material

The mutable struct [`Modia3D.Material`](@ref) defines
various visual properties of a geometrical object, such as *color* or *transparency*.
This data is used for visualization. Currently, there is no dictionary provided.


## Solid material

The dictionary [`Modia3D.solidMaterialPalette`](@ref) provides material constants of *one solid*.
The *key* is the *material name* as a string, and the *value* is an instance of the mutable struct
[`Modia3D.SolidMaterial`](@ref). This data is used, for example, to compute mass and inertia of an object
(with *density* and the object geometry), or to compute the spring constant
for a compliant contact (with *Youngs's modulus* and *Poisson's ratio*).
It is filled during the first usage of Modia3D from file `Modia3D/palettes/solidMaterials.json`.

### Content of solid material palette

```@repl
import Modia3D
Modia3D.listKeys(Modia3D.solidMaterialPalette)
```



## Contact pair material

The dictionary [`Modia3D.contactPairMaterialPalette`](@ref) provides
omaterial constants that are related to *two* solids that are in contact to each other,
for example the *coefficient of restitution*  between a `"Steel"` and an `"Aluminium"` object.
It is filled during the first usage of Modia3D from file `Modia3D/palettes/contactPairMaterials.json`.

### Content of contact pairs palette

```@repl
import Modia3D
Modia3D.listKeys(Modia3D.contactPairMaterialPalette)
```
