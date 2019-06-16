# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
# Content:
#
# Properties of contact materials



mutable struct ContactMaterialElastic <: Modia3D.AbstractContactMaterial
   c::Float64      # [N/m]       Spring constant of contact in normal direction (normalForce = c*relPosition)
   d::Float64      # [N*s/m]     Damping constant of contact in normal direction (normalForce = d*relVelocity)
   #d_w::Float64   # [N*m*s/rad] Rotational damping constant in normal direction (frictionTorque = d_w*relAngularVelocity)
   #mu0::Float64   #             Friction force in tangential direction (frictionForce = (mu0+mu1*relVelocity)*normalForce)
   mu1::Float64    #             frictionForce = (mu1*normalForce)*relTangentialVelocity
   #v_min::Float64 # [m/s]       Tangential velocity at which sliding friction force starts
   #w_min::Float64 # [rad/s]     Rel. angular velocity at which sliding friction torque starts

   function ContactMaterialElastic(; c = 1e9, d = 100.0, mu1=0.0)
      @assert(c > 0.0)
      @assert(d >= 0.0)
      #@assert(d_w >= 0.0)
      #@assert(mu0 >= 0.0)
      @assert(mu1 >= 0.0)
      #@assert(v_min > 0.0)
      #@assert(w_min > 0.0)
      new(c, d, mu1)
   end
end

#=
- `c` in [N/m]: Elastic material constant in normal direction (``\\gt 0``).
- `d` in [Ns/m]: Damping material constant in normal direction (``\\ge 0``).
- `mu_k`: Kinetic/sliding friction coefficient in tangential direction between two objects of the same material (``\\ge 0``).
- `v_min` in [m/s]: Absolute value of tangential velocity of current
                    object at which sliding friction force starts (``\\gt 0``).
- `E` in [N/m^2]: Young's modulus of material (``\\gt 0``).
- `nu`: Poisson's ratio of material (``0 \\lt nu \\lt 1``).
- `cof`: Coefficient of restitution between two objects of the same material (``0 \\le cof \\le 1``).
- `name::AbstractString`: Name of the material.
=#


"""
    material = ElasticContactMaterial(;name="", E=NaN, nu=NaN, cof=NaN, mu_k=NaN, 
                                       mu_r=NaN, v_small=0.01, w_small=v_small,)

Generate an `ElasticContactMaterial` object. If `name` is an empty string, the material
data (`E, nu, cof, mu_k, mu_r`) must be provided as input argument.
If `name` is not an empty string, the default material data is extracted from the
`Modia3D.solidMaterialPalette` dictionary using `name` as key
(available keys are for example `"Steel", "Aluminium", "DryWood"`).
Material data arguments that are additionally provided, overwrite the data from the
`solidMaterialPalette`. An argument with value `NaN` is ignored.

# Arguments

- `name::AbstractString`: Optional name of the material.
- `E` in [N/m^2]: Young's modulus of material (``\\gt 0``).
- `nu`: Poisson's ratio of material (``0 \\lt nu \\lt 1``).
- `cof`: Coefficient of restitution between two objects of the same material (``0 \\le cof \\le 1``).
- `mu_k`: Kinetic/sliding friction force coefficient between two objects of the same material (``\\ge 0``).
          The contact force in tangential contact surface direction is proportional to `mu_k` and to
          the normal force and acts in opposite direction to the relative tangential velocity.
- `mu_r`: Rotational friction torque coefficient between two objects of the same material (``\\ge 0``).
          The contact torque is proportional to `mu_r` and to the normal force and acts in 
          opposite direction to the relative angular velocity.
- `v_small` in [m/s]: Used for regularization when computing the unit vector in direction of 
          the relative tangential velocity ``\\vec{e}_t := \\vec{v}_{rel,t}/|\\vec{v}_{rel,t}|`` to avoid
          a division by zero if ``|\\vec{v}_{rel,t}| = 0`` (``\\gt 0``).
- `w_small` in [rad/s]: Used for regularization when computing the unit vector in direction of 
          the relative angular velocity ``\\vec{e}_{\\omega} := \\vec{\\omega}_{rel}/|\\vec{\\omega}_{rel}|`` to avoid
          a division by zero if ``|\\vec{\\omega}_{rel}| = 0`` (``\\gt 0``).


# Mathematical description of the response calculation

When two 3D objects penetrate each other with a penetration depth ``\\delta < 0``,
then a *contact force* and a *contact torque* is computed from the elastic contact materials
of the two objects in the following way
(the contact force law in normal direction is based on [^1], [^3], 
the remaining force law on [^2] with some extensions and corrections):


```math
\\begin{align}
f_{n,2}        &= \\max\\left(0, c_{res} \\cdot |\\delta| \\cdot (1 - d_{res} \\dot{\\delta}) \\right) \\\\
               & \\quad\\quad cor_{mean} = \\max(0.001, (cor_1 + cor_2)/2.0) \\\\
               & \\quad\\quad cor_{res} = cor_{mean} + (0.001 - cor_{mean}) \\cdot e^{log(0.01) |\\dot{\\delta}^-|/v_{small}} \\\\
               & \\quad\\quad d_{res} = \\frac{8(1-cor_{res})}{\\left(5 \\cdot cor_{res} \\cdot reg(|\\dot{\\delta}^-|, v_{small}) \\right)} \\\\
\\vec{f}_{t,2} &= -\\mu_{k,res} f_{n,2} \\frac{\\vec{v}_{rel,t}}{reg(|\\vec{v}_{rel,t}|, v_{small})}  \\\\[5mm]
\\vec{f}_2     &= f_{n,2} \\vec{e}_n + \\vec{f}_{t,2}  \\\\
\\vec{f}_1     &= -\\vec{f}_2 \\\\[5mm]
\\vec{\\tau}_2 &= -\\mu_{r,res} f_{n,2} \\frac{\\vec{\\omega}_{rel}}{reg(|\\vec{\\omega}_{rel}|,\\omega_{small})} \\\\
\\vec{\\tau}_1 &= -\\vec{\\tau}_2 \\\\[5mm]
\\end{align}
```

If at least one of the two 3D objects is a sphere, with ``r_i`` the sphere radius
and ``1/r_i = 0`` if object is not a sphere, and ``1/r = 1/r_1 + 1/r_2``, then 
the normal force ``f_{n,2}`` and the contact torque vector ``\\vec{\\tau}_2`` are
computed in the following way:

```math
\\begin{align}
       f_{n,2} &= \\max\\left(0, c_{res} \\cdot \\left(\\frac{4}{3} |\\delta| \\sqrt{r |\\delta|} \\right) \\cdot
                  (1 - d_{res} \\dot{\\delta}) \\right) \\\\
\\vec{\\tau}_2 &= -r \\cdot \\mu_{r,res} \\cdot f_{n,2} \\frac{\\vec{\\omega}_{rel}}{reg(|\\vec{\\omega}_{rel}|,\\omega_{small})}
\\end{align}
```

where

- ``\\vec{e}_n``: Unit contact normal vector from object 1 to object 2.
- ``f_{n,2}``: Value of normal force in direction of contact normal ``\\vec{e}_n`` acting on object 2 (``f_{n,2} \\ge 0``).
- ``\\vec{f}_{t,2}``: Contact force vector projected in tangential direction acting on object 2.
- ``\\vec{f}_1``: Contact force vector acting on object 1.
- ``\\vec{f}_2``: Contact force vector acting on object 2.
- ``\\vec{\\tau}_1``: Contact torque vector acting on object 1.
- ``\\vec{\\tau}_2``: Contact torque vector acting on object 2.
- ``\\delta``: Signed distance between object 1 and 2 in normal direction ``\\vec{e}_n``. 
               ``\\delta < 0`` if objects are penetrating each other.
- ``\\dot{\\delta}``: Signed relative velocity between object 1 and 2 in normal direction ``\\vec{e}_n``. 
- ``\\dot{\\delta}^-``: Value of ``\\dot{\\delta}`` when contact starts (``\\dot{\\delta}^- \\le 0``). 
- ``\\vec{v}_{rel,t}``: Relative velocity vector between object 1 and 2 projected in tangential direction
                        (``= \\vec{v}_2 - \\vec{v}_1`` and ``\\vec{v}_1, \\vec{v}_2`` are the absolute
                        velocities of the points on objects 1 and 2 that are in contact to each other).
- ``\\vec{\\omega}_{rel}``: Relative angular velocity vector between object 1 and 2.
                        (``= \\vec{\\omega}_2 - \\vec{\\omega}_1`` and ``\\vec{\\omega}_1, \\vec{\\omega}_2`` are the absolute
                        angular velocities of objects 1 and 2 that are in contact to each other).
- ``c_{res}``: Resultant elastic material constant in normal direction. This constant is computed
         from the constants ``c_1, c_2`` of the two contacting objects 1 and 2 as
         ``1/c_{res} = 1/c_1 + 1/c_2``. ``c_i`` is computed from the material properties as
         ``c_i = E_i/(1 - \\nu_i^2)`` where ``E_i`` is Young's modules and
         ``\\nu_i`` is Poisson's ratio of object i.
- ``d_{res}``: Resultant damping material constant in normal direction. This constant is computed
         from the resultant coefficient of restitution ``cor_{res}`` in equation (4) according to [^1].
- ``cor_{res}``: Resultant coefficient of restitution. This constant is computed from the
                 coeffients of restitutions of the two objects ``cor_1, cor_2`` according to 
                 equation (3).
- ``\\mu_{k,res}``: Resultant kinetic/sliding friction force coefficient in tangential direction. This constant is computed
         from the constants ``\\mu_{k,1}, \\mu_{k,2}`` of the two contacting objects 1 and 2 as
         ``\\mu_{k,res} = \\min(\\mu_{k,1},\\mu_{k,2})``.
- ``\\mu_{r,res}``: Resultant rotational friction torque coefficient. This constant is computed
         from the constants ``\\mu_{r,1}, \\mu_{r,2}`` of the two contacting objects 1 and 2 as
         ``\\mu_{r,res} = \\min(\\mu_{r,1},\\mu_{r,2})``.
- ``v_{small}``: Small velocity (used for regularization around ``|\\vec{v}_{rel,t}| = 0``).
- ``\\omega_{small}``: Small angular velocity (used for regularization around ``|\\vec{\\omega}_{rel}| = 0``).

If both objects are spheres or one of them is a sphere and the other contact surface is flat, then
``f_{n,2}`` of equation (11) is a reasonable contact force approximation consisting of Hertz'pressure for the elastic
contact force law and an approximation of the damping so that a similar response is achieved as if
an impulsive based contact law with a coefficient of restitution would be used, see [^1], [^3].
In all other cases, ``f_n`` is a very crude approximation of the normal contact force,
because it depends only on the penetration depth. A more realistic normal contact force
computation would require to estimate the penetration volume. 
Note, in the special case of equation (12), the rotational coefficient of friction ``\\mu_{r,res}``
can be interpreted as *rolling resistance coefficient*.

The ``\\max(..)`` operator in equation (1) is provided, in order
to guarantee that ``f_n`` is always a compressive and never a pulling force because
this would be unphysical.

The equations above need constants for object pairs. However, only material data
for the respective surfaces are provided and not for object pairs. 
The correct solution would be to apply, say, (1) a force on object 1 computed with the
material constants of object 1, (2) a force element on object 2 using the material
constants of object 2 and (3) connect the force elements of object 1 and 2 in series.
Due to the damping term this would require to solve an additional differential equation
to compute the actual contact point because there will be different penetrations of
object 1 and 2. This in turn would mean that the number of differential equations of the
system would depend on the number of objects that are in contact to each other.
To avoid this large complication, the following approximation is used: A resultant spring constant is computed
from the object data under the assumption of a series connections of two springs. For all other data,
mean values are used, with exception of the cofficients of frictions, where it is a better
approximation to use the coefficient of friction of the object that has the smallest value also
as resultant coefficient of friction.

Whenever the coefficient of restitution ``cor_i > 0``, then an object 2 jumping on an object 1 will
mathematically never come to rest, although this is unphysical. To fix this, the value of a coefficient of restitution
is reduced when the velocity at contact start becomes small. Furthermore, the coefficient of restitution
is restricted to not become smaller as a minimum value (= 0.001), in order to avoid a division by zero
when computing the resultant damping coefficient with equation (4). The characteristics of equation (3)
to compute the resultant coefficient of restitution is shown in the next figure:

![Resultant coefficient of restitution](../../resources/images/plot_cor.svg)


Various equations need a regularization to avoid a division by zero. This is accomplished with 
function ``reg(v_{abs}, v_{small})`` that has the following characteristics
(the function returns ``v_{abs}`` if ``v_{abs} \\ge v_{small}`` and otherwise returns
a third order polynomial with a minimum of ``v_{small}/3`` at ``v_{abs}=0`` and 
smooth first and second derivative at ``v_{abs} = v_{small}``):


![Regularization function](../../resources/images/plot_reg.svg)

The resultant damping coefficient ``d_{res}`` is basically computed from the 
resultant coefficient of restitution with equation (4) from [^1],
but with regularization of ``|\\dot{\\delta}^-|`` to avoid a division by zero.
The overall characteristics is shown in the next two figures:

![Regularization function](../../resources/images/plot_damping1.svg)

![Regularization function](../../resources/images/plot_damping2.svg)


The tangential force law is basically a sliding friction force
``\\vec{f}_t = -\\mu_k f_n \\vec{e}_t`` acting in opposite direction of the movement in the tangential plane.
The unit vector in direction of movement is ``\\vec{e}_t = \\vec{v}_t/|\\vec{v}_t|``.
The denominator is again regularized to avoid a division by zero when ``|\\vec{v}_t| = 0``.
The result is that the absolute value of the unit vector in direction of the movement
is approximated by the following smooth characteristics:

![Sliding friction force](../../resources/images/plot_friction1.svg)

This also means that the friction force is approximated by a steep characteristic
from zero force up to the sliding friction force. Currently, the stuck phase of friction is
not modelled. It is planned to include this important effect at some point in the future.

In a similar way the contact torque is computed from equation (8) using
also a regularization. This contact torque acts in opposite direction to the
relative angular velocity between the two contacting objects.


# Literature
[^1]: Paulo Flores, Margarida Machado, Miguel Silva, Jorge Martins (2011):
      [On the continuous contact force models for soft materials in 
      multibody dynamics](https://hal.archives-ouvertes.fr/hal-01333699).
      Multibody System Dynamics, Springer Verlag,
      Vol. 25, pp. 357-375. 10.1007/s11044-010-9237-4.

[^2]: Martin Otter, Hilding Elmqvist, José Díaz López (2005): 
      [Collision Handling for the Modelica MultiBody Library](https://modelica.org/events/Conference2005/online_proceedings/Session1/Session1a4.pdf). 
      Proceedings of the 4th International Modelica Conference 2005, 
      Gerhard Schmitz (Ed.), pages 45-53. 

[^3]: Luka Skrinjar, Janko Slavic, Miha Boltezar (2018): 
      [A review of continuous contact-force models in multibody dynamics](https://doi.org/10.1016/j.ijmecsci.2018.07.010). 
      International Journal of Mechanical Sciences, Volume 145,
      Sept., pages 171-187. 


# Example

```julia
# E=2.0e11, nu=0.3, cof=0.7, mu_k=0.5, mu_r=0.01 
material1 = ElasticContactMaterial(name="Steel") 

# E=2.0e11, nu=0.3, cof=0.8, mu_k=0.5, mu_r=0.01  
material2 = ElasticContactMaterial(name="Steel", cof=0.8) 
```
"""
mutable struct ElasticContactMaterial <: Modia3D.AbstractContactMaterial
   c::Float64      # [N/m]    Spring constant of contact in normal direction
   d::Float64      # [N*s/m]  Scaled damping constant of contact in normal direction
   mu_k::Float64   # []       Kinetic/sliding friction force coefficient
   mu_r::Float64   # []       Rotational friction torque coefficient
   v_small::Float64  # [m/s]    Absolute value of relative tangential velocity at which sliding friction force starts
   w_small::Float64  # [m/s]    Absolute value of relative angular velocity at which sliding friction torque starts

   function ElasticContactMaterial(; c=1e7, d=1.0, mu_k=0.1, mu_r=0.01, v_small=0.01, w_small=v_small)
      @assert(c > 0.0)
      @assert(d >= 0.0)
      @assert(mu_k >= 0.0)
      @assert(mu_r >= 0.0)
      @assert(v_small > 0.0)
      @assert(w_small > 0.0)
      new(c, d, mu_k, mu_r, v_small, w_small)
   end
end


function dampingFromCof(cof)
    @assert(cof >= 0.0 && cof <= 1.0)
    cof2 = max(cof, 0.01)
    return 8*(1-cof2)/(5*cof2)
end


function ElasticContactMaterialFromMaterialData(; E=20e9, nu=0.3, cof=0.7, mu_k=0.1, mu_r=0.01, v_small=0.01, w_small=v_small)
    @assert(E > 0.0)
    @assert(nu > 0.0 && nu < 1.0)
    @assert(cof >= 0.0 && cof <= 1.0)
    c = E/(1 - nu^2)
    d = dampingFromCof(cof) 
    return ElasticContactMaterial(c=E/(1 - nu^2), d=dampingFromCof(cof), mu_k=mu_k, mu_r=mu_r, v_small=v_small, w_small=w_small)
end


function ElasticContactMaterial(name::AbstractString; v_small=0.01, w_small=0.01)
   mat   = SolidMaterial(name)
   E     = mat.YoungsModulus
   nu    = mat.PoissonsRatio
   cof   = mat.coefficientOfRestitution
   mu_k  = mat.slidingFrictionCoefficient
   mu_t  = mat.rotationalFrictionCoefficient
   return ElasticContactMaterialFromMaterialData(E=E, nu=nu, cof=cof, mu_k=mu_k, mu_t=mu_t, v_small=v_small, w_small=w_small)
end



defaultContactMaterial() = ContactMaterialElastic()
