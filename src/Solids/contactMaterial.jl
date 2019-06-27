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



mutable struct ElasticContactMaterial <: Modia3D.AbstractContactMaterial
    c::Float64        # [N/m]   Spring constant of contact in normal direction
    cor::Float64      # []      Coefficient of restitution between two objects of the same material
    mu_k::Float64     # []      Kinetic/sliding friction force coefficient
    mu_r::Float64     # []      Rotational friction torque coefficient
    v_small::Float64  # [m/s]   Used for regularization when computing the unit vector in direction of
                      #         the relative tangential velocity to avoid a division by zero.
    w_small::Float64  # [rad/s] Used for regularization when computing the unit vector in direction of
                      #         the relative angular velocity to avoid a division by zero.

    function ElasticContactMaterial(;name="", E=NaN, nu=NaN, cor=NaN, mu_k=NaN, mu_r=NaN, v_small=0.01, w_small=v_small)
        @assert(v_small > 0.0)
        @assert(w_small > 0.0)
        if name == ""
            @assert(!isnan(E)    && E > 0.0)
            @assert(!isnan(nu)   && (nu > 0.0 && nu < 1.0))
            @assert(!isnan(cor)  && (cor >= 0.0 && cor <= 1.0))
            @assert(!isnan(mu_k) && mu_k >= 0.0)
            @assert(!isnan(mu_r) && mu_r >= 0.0)
            E2     = E
            nu2    = nu
            cor2   = cor
            mu_k2  = mu_k
            mu_r2  = mu_r
        else
            mat    = solidMaterialPalette[name]
            E2     = mat.YoungsModulus
            nu2    = mat.PoissonsRatio
            cor2   = mat.coefficientOfRestitution
            mu_k2  = mat.slidingFrictionCoefficient
            mu_r2  = mat.rotationalFrictionCoefficient
            if !isnan(E)
                @assert(E > 0.0)
                E2 = E
            end
            if !isnan(nu)
                @assert(nu > 0.0 && nu < 1.0)
                nu2 = nu
            end
            if !isnan(cor)
                @assert(cor >=0.0 && cor <= 1.0)
                cor2 = cor
            end
            if !isnan(mu_k)
                @assert(mu_k >= 0.0)
                mu_k2 = mu_k
            end
            if !isnan(mu_r)
                @assert(mu_r >= 0.0)
                mu_r2 = mu_r
            end
        end

        c = E2/(1 - nu2^2)
        new(c, cor2, mu_k2, mu_r2, v_small, w_small)
    end
end


"""
    material = ElasticContactMaterial2(name; vsmall=0.01, wsmall=0.01)

Generate an `ElasticContactMaterial` object from material `name` which is
used as key in dictionaries

- `Modia3D.solidMaterialPalette` to inquire constants `E, nu` and
- `Modia3D.solidMaterialPairsPalette` to inquire constants `cor, mu_k, mu_r`.

Available keys are for example `"Steel", "Aluminium", "DryWood", "BilliardBall"`.


# Arguments

- `name::AbstractString`: Name of the contact material (= key for material dictionaries).
- `vsmall::Float64` in [m/s]: Used for regularization when computing the unit vector in direction of
                              the relative tangential velocity (see below).
- `wsmall::Float64` in [rad/s]: Used for regularization when computing the unit vector in direction of
                                the relative angular velocity (see below).


# Material constants inquired from dictionaries

- `E::Float64` in [N/m^2]: Young's modulus of contact material (``\\gt 0``).
- `nu::Float64`: Poisson's ratio of contact material (``0 \\lt nu \\lt 1``).
- `cor::Float64`: Coefficient of restitution between *two objects* (``0 \\le cor \\le 1``).
- `mu_k::Float64`: Kinetic/sliding friction force coefficient between *two objects* (``\\ge 0``).
- `mu_r::Float64`: Rotational rolling resistance torque coefficient between *two objects* (``\\ge 0``).


# Mathematical description of the response calculation

When two 3D objects penetrate each other with a penetration depth ``\\delta > 0``
then a *contact force* and a *contact torque* is computed from the elastic contact materials
of the two objects in the following way
(the contact force law in normal direction is based on [^1], [^3],
the remaining force law on [^2] with some extensions and corrections):


```math
\\begin{align}
f_n        &= \\max\\left(0, c_{res} \\cdot c_{geo} \\cdot \\delta^{n_{geo}} \\cdot (1 + d \\cdot \\dot{\\delta}) \\right) \\\\
\\vec{f}_n &= f_n \\cdot \\vec{e}_n \\\\
\\vec{f}_t &= -\\mu_{k} \\cdot f_n \\cdot \\vec{e}_{t,reg} \\\\
\\vec{\\tau} &= -\\mu_{r} \\cdot \\mu_{r,geo}  \\cdot f_n \\cdot \\vec{e}_{\\omega,reg}
\\end{align}
```

where

- ``\\vec{e}_n``: Unit vector perpendicular to the surface of object 1 at the contact point, pointing outwards.
- ``\\vec{e}_{t,reg}``: Regularized unit vector in direction of the tangential relative velocity (see below).
- ``\\vec{e}_{\\omega,reg}``: Regularized unit vector in direction of the relative angular velocity (see below).
- ``f_n``: Value of normal contact force in direction of ``\\vec{e}_n`` acting on object 2 (``f_n \\ge 0``).
- ``\\vec{f}_n``: Vector of normal contact force acting on object 2.
- ``\\vec{f}_t``: Vector of sliding friction force acting on object 2 in opposite direction of the movement
                  in the tangential plane of the contact.
- ``\\vec{\\tau}``: Vector of rolling resistance torque acting on object 2, in opposite direction to the
                    relative angular velocity between the two contacting objects.
- ``\\delta``: Signed distance between object 1 and 2 in normal direction ``\\vec{e}_n``.
               ``\\delta > 0`` if objects are penetrating each other.
- ``\\dot{\\delta}``: Signed relative velocity between object 1 and 2 in normal direction ``\\vec{e}_n``.
- ``c_{res}``: Resultant elastic material constant in normal direction. This constant is computed
         from the constants ``c_1, c_2`` of the two contacting objects 1 and 2 as
         ``c_{res} = 1/\\left( 1/c_1 + 1/c_2 \\right)``. ``c_i`` is computed from the material properties as
         ``c_i = E_i/(1 - \\nu_i^2)`` where ``E_i`` is Young's modules and
         ``\\nu_i`` is Poisson's ratio of object i.
- ``c_{geo}``: Factor in ``f_n`` that is determined from the geometries of the
               two objects (see below).
- ``n_{geo}``: Exponent in ``f_n`` that is determined from the geometries of the
               two objects (see below).
- ``d(cor_{reg},\\dot{\\delta}^-)``: Damping coefficient in normal direction as a function of
         ``cor_{reg}`` and ``\\dot{\\delta}^-`` (see below).
- ``cor_{reg}``: Regularized coefficient of restitution between objects 1 and 2, see below.
- ``\\dot{\\delta}^-``: Value of ``\\dot{\\delta}`` when contact starts (``\\dot{\\delta}^- \\ge 0``).
- ``\\mu_k``: Kinetic/sliding friction force coefficient between objects 1 and 2.
- ``\\mu_r``: Rotational rolling resistance torque coefficient between objects 1 and 2.
- ``\\mu_{r,geo}``: Factor in ``\\vec{\\tau}`` that is determined from the geometries of the
                    two objects (see below).

The ``\\max(..)`` operator in equation (1) is provided, in order
to guarantee that ``f_n`` is always a compressive and never a pulling force because
this would be unphysical.

In special cases (for example sphere rolling on a plane), the rotational coefficient of friction ``\\mu_{r,res}``
can be interpreted as *rolling resistance coefficient*.

Coefficients ``c_{geo}, n_{geo}, \\mu_{r,geo}`` depend on the geometries of the objects
that are in contact. Only for spheres meaning values are provided based on Hertz' pressure,
because currently the collision handling in Modia3D does no provide enough information for other
geometries (``r_i`` is the radius of sphere ``i``):

| Object 1    | Object 2  | ``c_{geo}``                              | ``n_{geo}`` | ``\\mu_{r,geo}``    |
|:----------- |:--------- |:---------------------------------------- |:------------|:------------------- |
| Sphere      | Sphere    | ``\\frac{4}{3} \\sqrt{1/(1/r_1+1/r_2)}`` | ``1.5``     | ``1/(1/r_1+1/r_2)`` |
| Sphere      | no Sphere | ``\\frac{4}{3} \\sqrt{r_1}``             | ``1.5``     | ``r_1``             |
| no Sphere   | no Sphere | ``1``                                    | ``1.0``     | ``1.0``             |


# Regularized unit vectors

The unit vectors ``\\vec{e}_t, \\vec{e}_{\\omega}`` are undefined if the relative velocity and/or
the relative angular velocity vanish. They are therefore approximately calculated using utility
function ``reg(v_{abs}, v_{small})``. This function returns ``v_{abs}`` if ``v_{abs} \\ge v_{small}``
and otherwise returns a third order polynomial with a minimum of ``v_{small}/3`` at ``v_{abs}=0`` and
smooth first and second derivatives at ``v_{abs} = v_{small}``):

```math
reg(v_{abs}, v_{small}) = \\text{if}~ v_{abs} \\ge v_{small} ~\\text{then}~ v_{abs} ~\\text{else}~
                          \\frac{v_{abs}^2}{v_{small}}
                          \\left( 1 - \\frac{v_{abs}}{3v_{small}} \\right) + \\frac{v_{small}}{3}
```

Example for ``v_{small} = 0.1``:

![Regularization function](../../resources/images/plot_reg.svg)

With ``\\vec{v}_i`` the absolute velocity of the contact point of object ``i``,
and ``\\vec{\\omega}_i`` the absolute angular velocity of object ``i``,
the regularized unit vectors are calculated with function ``reg(...)``
in the following way:

```math
\\begin{align}
\\vec{e}_{t,reg}       =& \\frac{\\vec{v}_{rel,t}}{reg(|\\vec{v}_{rel,t}|, v_{small})}; \\quad
                          \\vec{v}_{rel} = \\vec{v}_2 - \\vec{v}_1; \\;
                          \\vec{v}_{rel,t} = \\vec{v}_{rel}  - (\\vec{v}_{rel} \\cdot \\vec{e}_n) \\vec{e}_n \\\\
\\vec{e}_{\\omega,reg} =& \\frac{\\vec{\\omega}_{rel}}{reg(|\\vec{\\omega}_{rel}|,\\omega_{small})}; \\quad
                          \\vec{\\omega}_{rel} = \\vec{\\omega}_2 - \\vec{\\omega}_1
\\end{align}
```

The effect is that the absolute value of a regularized unit vector
is approximated by the following smooth characteristics (and therefore the corresponding
friction force and contact torque have a similar characteristic)

![Sliding friction force](../../resources/images/plot_friction1.svg)


# Damping coefficient ``d(cor_{reg},\\dot{\\delta}^-)``

There are several proposal to compute the damping coefficient as a function
of the coefficient of restitution ``cor`` and the
velocity when contact starts ``\\dot{\\delta}^-``.
For a comparision of the different formulations see [^1].

Whenever the coefficient of restitution ``cor > 0``, then an object 2 jumping on an object 1 will
mathematically never come to rest, although this is unphysical. To fix this, the value of a
coefficient of restitution is reduced when the velocity at contact start becomes small.
Furthermore, the coefficient of restitution is restricted to not become smaller as a minimum value
``cor_{min} = 0.001`` in order to avoid a division by zero when computing the damping coefficient.
The following regularization function is used:

```math
cor_{reg} = cor_{lim} + (cor_{min} - cor_{lim}) \\cdot e^{log(0.01) |\\dot{\\delta}^-|/v_{small}}; \\;\\;
            cor_{lim} = \\max(cor_{min}, cor)
```

Examples of this characteristics are shown in the next figure:

![Regularized coefficient of restitution](../../resources/images/plot_cor.svg)


The damping coefficient ``d`` is basically computed with the formulation from [^1]
because a response calculation with impulses gives similar results for some experiments
as shown in [^3]. However, (a) instead of the coefficient of restitution, the regularized form
from above is used, (b) ``|\\dot{\\delta}^-|`` is regularized to avoid a division by zero,
and (c) the damping coefficient is limited to ``d_{max} = 1000`` to avoid an unphysical
strong creeping effect for collisions with small ``cor_{reg}`` values:

```math
d(cor_{reg},\\dot{\\delta}^-) = \\min(1000, \\frac{8(1-cor_{reg})}{\\left(5 \\cdot cor_{reg} \\cdot reg(|\\dot{\\delta}^-|, v_{small}) \\right)})
```

Examples of this characteristics are shown in the next two figures:

![Damping coefficient 1](../../resources/images/plot_damping1.svg)

![Damping coefficient 2](../../resources/images/plot_damping2.svg)


In the next figure the simulation of a bouncing ball is shown where the response
calculation is performed (a) with an impulse and (b) with the compliant force law above.
In both cases the coefficient of restitution `cof` is going to zero when ``|\\dot{\\delta}^-|``
becomes small. As can be seen in both cases a similar response appears:

![Bouncing ball](../../resources/images/plot_bouncingBall.svg)


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
import Modia3D

material1 = Modia3D.ElasticContactMaterial(name="Steel")
material2 = Modia3D.ElasticContactMaterial(name="DryWood", vsmall=0.1)
```
"""
mutable struct ElasticContactMaterial2 <: Modia3D.AbstractContactMaterial
    name::AbstractString  #         Name of contact material
    c::Float64            # [N/m]   Spring constant of contact in normal direction
    v_small::Float64       # [m/s]   Used for regularization when computing the unit vector in direction of
                          #         the relative tangential velocity to avoid a division by zero.
    w_small::Float64       # [rad/s] Used for regularization when computing the unit vector in direction of
                          #         the relative angular velocity to avoid a division by zero.

    function ElasticContactMaterial2(name; v_small=0.01, w_small=0.01)
        @assert(v_small > 0.0)
        @assert(w_small > 0.0)
        mat = solidMaterialPalette[name]
        E   = mat.YoungsModulus
        nu  = mat.PoissonsRatio
        @assert(E > 0.0)
        @assert(nu > 0.0 && nu < 1.0)

        c = E/(1 - nu^2)
        new(name, c, v_small, w_small)
    end
end



regularize(absv,vsmall) = absv >= vsmall ? absv : absv*(absv/vsmall)*(1.0 - (absv/vsmall)/3.0) + vsmall/3.0


function resultantCoefficientOfRestitution(cor1,cor2,abs_vreln,vsmall)
    @assert(cor1 >= 0.0 && cor1 <= 1.0)
    @assert(cor2 >= 0.0 && cor2 <= 1.0)
    @assert(abs_vreln >= 0.0)
    @assert(vsmall > 0)

    cor_min  = 0.001
    cor_mean = max(cor_min, min(cor1,cor2))   # (cor1 + cor2)/2.0)
    cor_res  = cor_mean + (cor_min - cor_mean)*exp(log(0.01)*(abs_vreln/vsmall))
    return cor_res
end


function resultantDampingCoefficient(cor1, cor2, abs_vreln, vsmall)
    @assert(cor1 >= 0.0 && cor1 <= 1.0)
    @assert(cor2 >= 0.0 && cor2 <= 1.0)
    @assert(abs_vreln >= 0.0)
    @assert(vsmall > 0)

    cof_res = resultantCoefficientOfRestitution(cor1,cor2,abs_vreln,vsmall)
    d_res   = 8.0*(1.0 - cof_res)/(5*cof_res*regularize(abs_vreln,vsmall))
    return min(d_res,1000.0)
end


# both functions are for new contact law
resultantCoefficientOfRestitution(cor,abs_vreln,vsmall;cor_min=0.01) =
    cor + (cor_min - cor)*exp(log(0.01)*(abs_vreln/vsmall))

function resultantDampingCoefficient(cor, abs_vreln, vsmall; cor_min=0.01, d_max=1000.0)
    @assert(cor >= 0.0 && cor <= 1.0)
    @assert(abs_vreln >= 0.0)
    @assert(vsmall  > 0.0)
    @assert(cor_min > 0.0)
    @assert(d_max   > 0.0)

    cor_res = resultantCoefficientOfRestitution(cor,abs_vreln,vsmall;cor_min=cor_min)
    return min(d_max, 8.0*(1.0 - cor_res)/(5*cor_res*regularize(abs_vreln,vsmall)))
end


defaultContactMaterial() = ContactMaterialElastic()
