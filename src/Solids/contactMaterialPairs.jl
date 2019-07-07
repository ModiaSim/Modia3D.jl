
struct KeyCollisionMaterialPairs <: Modia3D.AbstractKeys
    material1::String
    material2::String
    function KeyCollisionMaterialPairs(material1::String, material2::String)
        minMaterial = min(material1,material2)
        if minMaterial != material1
            return new(material2, material1)
        else
            return new(material1, material2)
        end
    end
end


function Base.:isequal(keyA::KeyCollisionMaterialPairs, keyB::KeyCollisionMaterialPairs)
    if (keyA.material1 == keyB.material1 && keyA.material2 == keyB.material2)
        return true
    end
    return false
end


mutable struct CommonCollisionProperties <: Modia3D.AbstractContactMaterial
    cor::Float64      # []      Coefficient of restitution between two objects
    mu_k::Float64     # []      Kinetic/sliding friction force coefficient between two objects
    mu_r::Float64     # []      Rotational friction torque coefficient between two objects
    function CommonCollisionProperties(cor, mu_k, mu_r)
        @assert(cor  >= 0.0)
        @assert(mu_k >= 0.0)
        @assert(mu_r >= 0.0)
        new(cor, mu_k, mu_r)
    end
end


"""
    const solidMaterialPairsPalette

Material for contact pairs
"""
const solidMaterialPairsPalette = Dict{KeyCollisionMaterialPairs, CommonCollisionProperties}()

solidMaterialPairsPalette[KeyCollisionMaterialPairs("Steel", "Steel")]         = CommonCollisionProperties(0.7, 0.5, 0.001)

#=
solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardBall")]         = CommonCollisionProperties(1.0, 0.0, 0.0)
solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardTable")]   = CommonCollisionProperties(0.0, 0.6, 0.02)

solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardCushion")] = CommonCollisionProperties(0.8, 0.0, 0.0)
=#

solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardBall")]         = CommonCollisionProperties(1.0, 0.0, 0.0)
solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardTable")]   = CommonCollisionProperties(0.0, 0.8, 0.01)

solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardCushion")] = CommonCollisionProperties(0.8, 0.0, 0.3)

solidMaterialPairsPalette[KeyCollisionMaterialPairs("DryWood", "DryWood")] = CommonCollisionProperties(0.5, 0.0, 0.0)

getCommonCollisionProperties(obj1, obj2) = nothing

function getCommonCollisionProperties(mat1::ElasticContactMaterial2, mat2::ElasticContactMaterial2)
    values = get(solidMaterialPairsPalette, KeyCollisionMaterialPairs(mat1.name, mat2.name), false)
    if values != false
        return values
    end
    return nothing
end

getCommonCollisionProperties(name1::AbstractString, name2::AbstractString) =
                     solidMaterialPairsPalette[KeyCollisionMaterialPairs(name1,name2)]



"""
    m = ElasticContactPairMaterial2(m1::ElasticContactMaterial2,
                                    m2::ElasticContactMaterial2,
                                    delta_dot_initial::Float64)

Generate an `ElasticContactPairMaterial2 < AbstractContactPairMaterial` object.
"""
mutable struct ElasticContactPairMaterial2 <: Modia3D.AbstractContactPairMaterial
    c_res::Float64
    d_res::Float64
    mu_k::Float64
    mu_r::Float64
    vsmall::Float64
    wsmall::Float64

    function ElasticContactPairMaterial2(m1::ElasticContactMaterial2, m2::ElasticContactMaterial2, delta_dot_initial::Float64)
        collMaterial = getCommonCollisionProperties(m1.name, m2.name)
        c_res  = m1.c*m2.c/(m1.c + m2.c)
        vsmall = (m1.vsmall + m2.vsmall)/2
        wsmall = (m1.wsmall + m2.wsmall)/2
        mu_k   = collMaterial.mu_k
        mu_r   = collMaterial.mu_r
        d_res  = Modia3D.resultantDampingCoefficient(collMaterial.cor, abs(delta_dot_initial), vsmall)

        new(c_res, d_res, mu_k, mu_r, vsmall, wsmall)
    end
end

"""
    material = ContactPairMaterial(;coefficientOfRestitution=0.0, slidingFrictionCoefficient=0.0,
                                    rotationalResistanceCoefficient=0.0, vsmall=0.01, wsmall=0.01)

Generates a `ContactPairMaterial` object by providing the material properties of
two objects that are in contact to each other.

# Keyword Arguments
- coefficientOfRestitution: Coefficient of restitution between two objects (=0: inelastic ... =1: fully elastic).
- slidingFrictionCoefficient: Kinetic/sliding friction force coefficent between two objects (`>= 0.0`).
- rotationalResistanceCoefficient: Rotational resistance torque coefficient between two objects (`>= 0.0`).
- `vsmall` in [m/s]: Used for regularization when computing the unit vector in direction of
                     the relative tangential velocity to avoid a division by zero.
- `wsmall` in [rad/s]: Used for regularization when computing the unit vector in direction
                       of the relative angular velocity to avoid a division by zero.

# Example
```julia
import Modia3D
mat = Modia3D.ContactPairMaterial(coefficientOfRestitution=0.5)
```
"""
mutable struct ContactPairMaterial
    coefficientOfRestitution::Float64
    slidingFrictionCoefficient::Float64
    rotationalResistanceCoefficient::Float64
    vsmall::Float64
    wsmall::Float64

    function ContactPairMaterial(; coefficientOfRestitution=0.0,
                                   slidingFrictionCoefficient=0.0,
                                   rotationalResistanceCoefficient=0.0,
                                   vsmall=0.01,
                                   wsmall=0.01)
        @assert(coefficientOfRestitution >= 0.0)
        @assert(coefficientOfRestitution <= 1.0)
        @assert(slidingFrictionCoefficient >= 0.0)
        #@assert()
    end

end