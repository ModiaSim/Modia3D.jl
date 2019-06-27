
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


struct CommonCollisionProperties <: Modia3D.AbstractContactMaterial
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


const solidMaterialPairsPalette = Dict{KeyCollisionMaterialPairs, CommonCollisionProperties}()

solidMaterialPairsPalette[KeyCollisionMaterialPairs("Steel", "Steel")]         = CommonCollisionProperties(0.7, 0.5, 0.001)

solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardBall")]         = CommonCollisionProperties(1.0, 0.0, 0.0)
solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardTable")]   = CommonCollisionProperties(0.0, 0.6, 0.02)

solidMaterialPairsPalette[KeyCollisionMaterialPairs("BilliardBall", "BilliardCushion")] = CommonCollisionProperties(0.8, 0.0, 0.0)

getCommonCollisionProperties(obj1, obj2) = nothing

function getCommonCollisionProperties(mat1::ElasticContactMaterial2, mat2::ElasticContactMaterial2)
    values = get(solidMaterialPairsPalette, KeyCollisionMaterialPairs(mat1.name, mat2.name), false)
    if values != false
        return values
    end
    return nothing
end
