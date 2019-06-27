
# Materials

Modia3D uses various materials for different kind of purposes. In this section an overview
of the material data and material palettes is given, as well as links to more detailed
information. The following materia data is supported

- [Color material](@ref): Color names and associated RGB values.

- [Visual material](@ref): Visualization properties of a geometrical object, such as *color* or *transparency*.

- [Solid material](@ref): Material constants of **one** solid, such as
  *density* or *Young's modulus*.

- [Contact material](@ref): The dictionary [`Modia3D.contactMaterialPalette`](@ref) provides
  material constants that are related to **two** solids that are in contact to each other,
  such as the *coefficient of restitution*  between a `"Steel"` and an `"Aluminium"` object.


## Color material

 Color can be defined by a name, such as `Red`, and this
 name is associated with an Int-vector where the entries correspond to RGB values.
 Default color definitions are available via the [`Modia3D.colorPalette`](@ref)
 dictionary, where the color name is used as key and which is filled during the first
 usage of Modia3D from file `Modia3D/src/Graphics/colors.json`.

### Example:

```@repl
import Modia3D
sort(Modia3D.colorPalette)              # print dictionary (sorted)
red_rgb = Modia3D.colorPalette["Red"]   # get rgb values for "Red"
```


## Visual material

Visualization properties of a geometrical object
[Visual material](@ref): Visual propThe mutable struct [`Modia3D.Material`](@ref) defines
  various visual properties of a geometrical object, such as *color* or *transparency*.
  This data is used for visualization.
  Currently, there is no dictionary provided.

## Solid material

The dictionary [`Modia3D.solidMaterialPalette2`](@ref) provides material constants of **one** solid.
The key is the *material name* as a string, and the data is an instance of the mutable struct
[`Modia3D.SolidMaterial2`](@ref). This data is used, for example, to compute mass and inertia of an object
(with *density* and the object geometry), or to compute the spring constant
for a compliant contact (with *Youngs's modulus* and *Poisson's ratio*).

### Example:

```@repl
import Modia3D
Modia3D.listKeys(Modia3D.solidMaterialPalette2)      # print keys (sorted)
d_steel = Modia3D.solidMaterialPalette2["Steel"].density   # get density
```

## Contact material

- [Contact material](@ref): The dictionary [`Modia3D.contactMaterialPalette`](@ref) provides
  material constants that are related to **two** solids that are in contact to each other,
  such as the *coefficient of restitution*  between a `"Steel"` and an `"Aluminium"` object.


The dictionary [`Modia3D.contactMaterialPalette`](@ref) provides material constants
that are related to **two** solids that are in contact to each other.
The key is a struct consisting of two *material names* as strings and the data
is an instance of the mutable struct [`Modia3D.ContactMaterial`](@ref)
that has the following elements:

- `coefficientOfRestitution`: Coefficient of restitution between *two objects* (between 0..1), see [Wikipedia](https://en.wikipedia.org/wiki/Coefficient_of_restitution).
- `kineticFrictionCoefficient`: Kinetic/sliding friction coefficient between *two objects* (``\ge 0``), see [Wikipedia](https://en.wikipedia.org/wiki/Friction).
- `resistanceTorqueCoefficient`: Rotational rolling resistance torque coefficient between *two objects* (``\ge 0``). The contact torque is
  proportional to this coefficient and is directed in the opposite direction of the relative
  angular velocity between the two contacting objects.
- `vsmall` [m/s]: Used for regularization when computing the unit vector in direction of
  the relative tangential velocity (``> 0``).
- `wsmall` [rad/s]: Used for regularization when computing the unit vector in direction of
  the relative angular velocity (``> 0``).
