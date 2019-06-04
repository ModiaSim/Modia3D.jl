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


"""
    material1 = ElasticContactMaterial(c=1e7, d=1.0, mu_k=0.1, v_min=0.01)
    material2 = ElasticContactMaterialFromMaterialData(E=2e11, nu=0.3, cof=0.7, mu_k=0.1, v_min=0.01)
    material3 = ElasticContactMaterialFromMaterialName(name="Steel")

Generate an `ElasticContactMaterial` object. This object is used to 
compute an elastic response calculation between two 3D objects
in case the two objects are penetrating each other, that is if ``\\delta < 0``,
using the following force law:

```math
\\begin{align}
f_n        &= \\max\\left(0, c_{mean} \\cdot |\\delta| \\left(1 + d_{mean} \\frac{\\dot{\\delta}}{\\dot{\\delta}^-} \\right) \\right) \\\\
\\vec{f}_t &= \\mu_{k,mean} f_n \\frac{\\vec{v}_t}{|\\vec{v}_t| + v_{min} e^{-k|\\vec{v}_t|}} \\quad \\# \\; k = -log(0.01)/v_{min} \\\\
\\vec{f}_1 &= f_n \\vec{e}_n + \\vec{f}_t  \\\\
\\vec{f}_2 &= -\\vec{f}_1
\\end{align}
```

If at least one of the two 3D objects is a sphere, with ``r_i`` the sphere radius
and ``1/r_i = 0`` if object is not a sphere, and ``1/r = 1/r_1 + 1/r_2``, then 
the normal force is computed in the following way:

```math
f_n = \\max\\left(0, c_{mean} \\cdot \\left(\\frac{4}{3} |\\delta| \\sqrt{r |\\delta|} \\right) \\cdot
      \\left(1 + d_{mean} \\frac{\\dot{\\delta}}{\\dot{\\delta}^-} \\right) \\right)
```

where

- ``\\vec{e}_n``: Unit contact normal vector from object 1 to object 2.
- ``f_n``: Value of normal force in direction of contact normal ``\\vec{e}_n`` acting on object 1 (``f_n \\ge 0``).
- ``\\vec{f}_t``: Contact force vector projected in tangential direction acting on object 1.
- ``\\vec{f}_1``: Contact force vector acting on object 1.
- ``\\vec{f}_2``: Contact force vector acting on object 2.
- ``\\delta``: Signed distance between object 1 and 2. ``\\delta < 0`` if objects are penetrating each other.
- ``\\dot{\\delta}``: Signed relative velocity between object 1 and 2 in normal direction. 
- ``\\dot{\\delta}^-``: Value of ``\\dot{\\delta}`` when contact starts. 
- ``\\vec{v}_t``: Relative velocity vector between object 1 and 2 projected onto tangential direction.
- ``c_{mean}``: Elastic material constant in normal direction. This constant is computed
         from the constants ``c_1, c_2`` of the two contacting objects 1 and 2 as
         ``1/c_{mean} = 1/c_1 + 1/c_2``. ``c_i`` can be computed from the material properties as
         ``c_i = E_i/(1 - \\nu_i^2)`` where ``E_i`` is Young's modules and
         ``\\nu_i`` is Poisson's ratio of object i.
- ``d_{mean}``: Damping material constant in normal direction. This constant is computed
         from the constants ``d_1, d_2`` of the two contacting objects 1 and 2 as
         ``d_{mean} = (d_1+d_2)/2``.
         ``d_i`` can be computed from the *coefficient of restitution* ``cor_i`` as
         ``d_i=8(1-e_i)/(5e_i); e_i = max(cor_i,0.01)`` according to [^1].
- ``\\mu_{k,mean}``: Kinetic/sliding friction coefficient in tangential direction. This constant is computed
         from the constants ``\\mu_{k,1}, \\mu_{k,2}`` of the two contacting objects 1 and 2 as
         ``\\mu_{k,mean} = \\min(\\mu_{k,1},\\mu_{k,2})``.
- ``v_{min}``: Absolute value of tangential velocity at which sliding friction force starts.
               This constant is computed from the constants ``v_{min,1}, v_{min,2}``
               of the two contacting objects 1 and 2 as
               ``v_{min} = (v_{min,1}+v_{min,2})/2``.

The contact force law in normal direction is based on [^1], [^3], the remaining force law on [^2].


# Arguments

- `c` in [N/m]: Elastic material constant in normal direction (``\\gt 0``).
- `d` in [Ns/m]: Damping material constant in normal direction (``\\ge 0``).
- `mu_k`: Kinetic/sliding friction coefficient in tangential direction between two objects of the same material (``\\ge 0``).
- `v_min` in [m/s]: Absolute value of tangential velocity of current
                    object at which sliding friction force starts (``\\gt 0``).
- `E` in [N/m^2]: Young's modulus of material (``\\gt 0``).
- `nu`: Poisson's ratio of material (``0 \\lt nu \\lt 1``).
- `cof`: Coefficient of restitution between two objects of the same material (``0 \\le cof \\le 1``).
- `name::String`: Name of the material.


# Discussion

If both objects are spheres or one of them is a sphere and the other contact surface is flat, then
``f_n`` is a reasonable contact force approximation consisting of Hertz'pressure for the elastic
contact force law and an approximation of the damping so that a similar response is achieved as if
an impulsive based contact law with a coefficient of restitution would be used, see [^1], [^3].

In all other cases, ``f_n`` is a very crude approximation of the normal contact force,
because it depends only on the penetration depth. A more realistic normal contact force
computation would require to estimate the penetration volume. 

The ``\\max(..)`` operator in equation (1) is provided, in order
to guarantee that ``f_n`` is always a compressive and never a pulling force because
this would be unphysical.

The tangential force law is basically just a sliding friction force
``|\\vec{f}_t| = \\mu_k f_n`` acting in opposite direction of the movement in the tangential plane.
The stuck phase is not modelled, but is approximated by a steep characteristic
from zero force up to the sliding friction force, see next figure:

![Sliding friction force](../../resources/images/plot_friction1.svg)



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
"""
mutable struct ElasticContactMaterial <: Modia3D.AbstractContactMaterial
   c::Float64      # [N/m]    Spring constant of contact in normal direction
   d::Float64      # [N*s/m]  Scaled damping constant of contact in normal direction
   mu_k::Float64   # []       Kinetic/sliding friction force coefficient
   v_min::Float64  # [m/s]    Absolute value of tangential velocity at which sliding friction force starts

   function ElasticContactMaterial(; c=1e7, d=1.0, mu_k=0.1, v_min=0.01)
      @assert(c > 0.0)
      @assert(d >= 0.0)
      @assert(mu_k >= 0.0)
      @assert(v_min > 0.0)
      new(c, d, mu_k, v_min)
   end
end


function dampingFromCof(cof)
    @assert(cof >= 0.0 && cof <= 1.0)
    cof2 = max(cof, 0.01)
    return 8*(1-cof2)/(5*cof2)
end


function ElasticContactMaterialFromMaterialData(E=20e9, nu=0.3, cof=0.7, mu_k=0.1, v_min=0.01)
    @assert(E > 0.0)
    @assert(nu > 0.0 && nu < 1.0)
    @assert(cof >= 0.0 && cof <= 1.0)
    c = E/(1 - nu^2)
    d = dampingFromCof(cof) 
    return ElasticContactMaterial(c=E/(1 - nu^2), d=dampingFromCof(cof), mu_k=mu_k, v_min=v_min)
end


function ElasticContactMaterialFromMaterialName(name::String="Steel")
   mat   = SolidMaterial(name)
   E     = mat.YoungsModulus
   nu    = mat.PoissonsRatio
   cof   = mat.coefficientOfRestitution
   mu_k  = mat.slidingFrictionCoefficient
   v_min = mat.v_min 
   return ElasticContactMaterialFromMaterialData(E=E, nu=nu, cof=cof, mu_k=mu_k, v_min=v_min)
end



defaultContactMaterial() = ContactMaterialElastic()
