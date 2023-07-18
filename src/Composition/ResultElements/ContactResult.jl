
"""
    result = ContactResult(; object1, object2, objectCoordinateRef)

Return a `result` providing results of elastic MPR contacts between
`object1::`[`Object3D`](@ref) and `object2::`[`Object3D`](@ref).
If `objectCoordinateRef::`[`Object3D`](@ref) is defined, all vector
results are resolved in `objectCoordinateRef`, otherwise in world.

# Results

- `penetration` is the normal contact penetration (positive in case of contact).
- `penetrationVelocity` is the normal contact penetration velocity (positive for compression).
- `tangentialVelocity` is the absolute value of the tangential relative velocity.
- `angularVelocity` it the absolute value of the relative angular velocity.
- `normalForce` is the normal contact force (positive for pressure).
- `tangentialForce` is the absolute value of the tangential contact force.
- `torque` is the absolute value of the contact torque.
- `positionVector` is the absolute position vector of the contact point, resolved in `objectCoordinateRef`.
- `normalVector` is the unit vector in contact normal direction, pointing into `object1`, resolved in `objectCoordinateRef`.
- `forceVector` is the total contact force vector acting at the contact point on object1, resolved in `objectCoordinateRef`.
   On `object2` the same force vector is applied in inverse direction.
- `torqueVector` is the total contact torque vector acting at the contact point on object1, resolved in `objectCoordinateRef`.
   On `object2` the same torque vector is applied in inverse direction.
"""
mutable struct ContactResult{F <: Modia3D.VarFloatType} <: Modia3D.AbstractResultElement

    path::String

    object1::Object3D{F}
    object2::Object3D{F}
    objectCoordinateRef::Union{Object3D{F}, Nothing}

    penetrationResultIndex::Int
    penetrationVelocityResultIndex::Int
    tangentialVelocityResultIndex::Int
    angularVelocityResultIndex::Int
    normalForceResultIndex::Int
    tangentialForceResultIndex::Int
    torqueResultIndex::Int
    positionVectorResultIndex::Int
    normalVectorResultIndex::Int
    forceVectorResultIndex::Int
    torqueVectorResultIndex::Int

    function ContactResult{F}(; path::String = "",
                                object1::Object3D{F},
                                object2::Object3D{F},
                                objectCoordinateRef::Union{Object3D{F}, Nothing}=nothing ) where F <: Modia3D.VarFloatType
        return new(path, object1, object2, objectCoordinateRef)
    end
end
ContactResult(; kwargs...) = ContactResult{Float64}(; kwargs...)


function initializeResultElement(model::Modia.InstantiatedModel{F,TimeType}, result::ContactResult{F}) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    result.penetrationResultIndex         = Modia.new_w_segmented_variable!(model, result.path*".penetration"        , F(0), "m")
    result.penetrationVelocityResultIndex = Modia.new_w_segmented_variable!(model, result.path*".penetrationVelocity", F(0), "m/s")
    result.tangentialVelocityResultIndex  = Modia.new_w_segmented_variable!(model, result.path*".tangentialVelocity" , F(0), "m/s")
    result.angularVelocityResultIndex     = Modia.new_w_segmented_variable!(model, result.path*".angularVelocity"    , F(0), "rad/s")
    result.normalForceResultIndex         = Modia.new_w_segmented_variable!(model, result.path*".normalForce"        , F(0), "N")
    result.tangentialForceResultIndex     = Modia.new_w_segmented_variable!(model, result.path*".tangentialForce"    , F(0), "N")
    result.torqueResultIndex              = Modia.new_w_segmented_variable!(model, result.path*".torque"             , F(0), "N*m")
    result.positionVectorResultIndex      = Modia.new_w_segmented_variable!(model, result.path*".positionVector"     , SVector{3,F}(0, 0, 0), "m")
    result.normalVectorResultIndex        = Modia.new_w_segmented_variable!(model, result.path*".normalVector"       , SVector{3,F}(0, 0, 0))
    result.forceVectorResultIndex         = Modia.new_w_segmented_variable!(model, result.path*".forceVector"        , SVector{3,F}(0, 0, 0), "N")
    result.torqueVectorResultIndex        = Modia.new_w_segmented_variable!(model, result.path*".torqueVector"       , SVector{3,F}(0, 0, 0), "N*m")

    return nothing
end

function evaluateResultElement(model::Modia.InstantiatedModel{F,TimeType}, scene::Modia3D.Composition.Scene{F}, result::ContactResult{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    (contactPair, converse) = getElasticContactPair(scene, result.object1, result.object2)
    if !isnothing(contactPair)
        penetration = contactPair.results.penetration
        penetrationVelocity = contactPair.results.penetrationVelocity
        tangentialVelocity = contactPair.results.tangentialVelocity
        angularVelocity = contactPair.results.angularVelocity
        normalForce = contactPair.results.normalForce
        tangentialForce = contactPair.results.tangentialForce
        torque = contactPair.results.torque
        positionVector = contactPair.results.positionVector
        normalVector = contactPair.results.normalVector
        forceVector = contactPair.results.forceVector
        torqueVector = contactPair.results.torqueVector
        if converse
            normalVector = -normalVector
            forceVector = -forceVector
            torqueVector = -torqueVector
        end
        if !isnothing(result.objectCoordinateRef)
            positionVector = result.objectCoordinateRef.R_abs * positionVector
            normalVector = result.objectCoordinateRef.R_abs * normalVector
            forceVector = result.objectCoordinateRef.R_abs * forceVector
            torqueVector = result.objectCoordinateRef.R_abs * torqueVector
        end
    else
        penetration = F(0)
        penetrationVelocity = F(0)
        tangentialVelocity = F(0)
        angularVelocity = F(0)
        normalForce = F(0)
        tangentialForce = F(0)
        torque = F(0)
        positionVector = SVector{3,F}(0, 0, 0)
        normalVector = SVector{3,F}(0, 0, 0)
        forceVector = SVector{3,F}(0, 0, 0)
        torqueVector = SVector{3,F}(0, 0, 0)
    end

    if Modia.storeResults(model)
        Modia.copy_w_segmented_value_to_result(model, result.penetrationResultIndex, penetration)
        Modia.copy_w_segmented_value_to_result(model, result.penetrationVelocityResultIndex, penetrationVelocity)
        Modia.copy_w_segmented_value_to_result(model, result.tangentialVelocityResultIndex, tangentialVelocity)
        Modia.copy_w_segmented_value_to_result(model, result.angularVelocityResultIndex, angularVelocity)
        Modia.copy_w_segmented_value_to_result(model, result.normalForceResultIndex, normalForce)
        Modia.copy_w_segmented_value_to_result(model, result.tangentialForceResultIndex, tangentialForce)
        Modia.copy_w_segmented_value_to_result(model, result.torqueResultIndex, torque)
        Modia.copy_w_segmented_value_to_result(model, result.positionVectorResultIndex, positionVector)
        Modia.copy_w_segmented_value_to_result(model, result.normalVectorResultIndex, normalVector)
        Modia.copy_w_segmented_value_to_result(model, result.forceVectorResultIndex, forceVector)
        Modia.copy_w_segmented_value_to_result(model, result.torqueVectorResultIndex, torqueVector)
    end

    return nothing
end

function terminateResultElement(result::ContactResult{F}) where F <: Modia3D.VarFloatType
    return nothing
end
