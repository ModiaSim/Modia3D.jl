# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Solids (Modia3D/Solids/_module.jl)
#
# Content:
#
# Properties of contact materials


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
    vsmall::Float64       # [m/s]   Used for regularization when computing the unit vector in direction of
                          #         the relative tangential velocity to avoid a division by zero.
    wsmall::Float64       # [rad/s] Used for regularization when computing the unit vector in direction of
                          #         the relative angular velocity to avoid a division by zero.

    function ElasticContactMaterial2(name; vsmall=0.01, wsmall=0.01)
        @assert(vsmall > 0.0)
        @assert(wsmall > 0.0)
        mat = solidMaterialPalette[name]
        E   = mat.YoungsModulus
        nu  = mat.PoissonsRatio
        @assert(E > 0.0)
        @assert(nu > 0.0 && nu < 1.0)

        c = E/(1 - nu^2)
        new(name, c, vsmall, wsmall)
    end
end
