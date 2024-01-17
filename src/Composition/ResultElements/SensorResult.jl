
"""
    result = SensorResult(; object, objectOrigin, objectCoordinateRef, objectObserveRef)

Return a `result` providing kinematic measurements of
`object::`[`Object3D`](@ref) relative to
`objectOrigin::`[`Object3D`](@ref), resolved in coordinates of
`objectCoordinateRef::`[`Object3D`](@ref) and observed in
`objectObserveRef::`[`Object3D`](@ref).

In case of `objectOrigin` is undefined, world is used as sensor
origin, i.e. results are absolute measurements. In case of
`objectCoordinateRef` is undefined, `objectOrigin` is used as coordinate
reference frame. In case of `objectObserveRef = nothing`,
`objectCoordinateRef` is used as observer reference frame (so that
velocity and acceleration results are consistent time derivatives of
position results).

# Results
- `translation` is the position vector from `objectOrigin` to `object`.
- `velocity` is the translational velocity vector from `objectOrigin` to `object`, observed in `objectObserveRef`.
- `acceleration` is the translational acceleration vector from `objectOrigin` to `object`, observed in `objectObserveRef`.
- `rotation` contains the [Cardan (Tait–Bryan) angles](https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence) (rotation sequence x-y-z) from `objectOrigin` to `object`.
- `angularVelocity` is the rotational velocity vector from `objectOrigin` to `object`.
- `angularAcceleration` is the rotational acceleration vector from `objectOrigin` to `object`, observed in `objectObserveRef`.
- `distance` is the point-to-point distance between `objectOrigin` and `object`.
- `distanceVelocity` is the point-to-point velocity between `objectOrigin` and `object`.
- `distanceAcceleration` is the point-to-point acceleration between `objectOrigin` and `object`.
"""
mutable struct SensorResult{F <: Modia3D.VarFloatType} <: Modia3D.AbstractResultElement

    path::String

    object::Object3D{F}
    objectOrigin::Union{Object3D{F}, Nothing}
    objectCoordinateRef::Union{Object3D{F}, Nothing}
    objectObserveRef::Union{Object3D{F}, Nothing}

    translationResultIndex::Int
    velocityResultIndex::Int
    accelerationResultIndex::Int
    rotationResultIndex::Int
    angularVelocityResultIndex::Int
    angularAccelerationResultIndex::Int
    distanceResultIndex::Int
    distanceVelocityResultIndex::Int
    distanceAccelerationResultIndex::Int

    function SensorResult{F}(; path::String = "",
                               object::Object3D{F},
                               objectOrigin::Union{Object3D{F}, Nothing}=nothing,
                               objectCoordinateRef::Union{Object3D{F}, Nothing}=objectOrigin,
                               objectObserveRef::Union{Object3D{F}, Nothing}=objectCoordinateRef ) where F <: Modia3D.VarFloatType
        return new(path, object, objectOrigin, objectCoordinateRef, objectObserveRef)
    end
end
SensorResult(; kwargs...) = SensorResult{Float64}(; kwargs...)


function initializeResultElement(model::Modia.InstantiatedModel{F,TimeType}, result::SensorResult{F}) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    result.object.computeAcceleration = true
    if (!isnothing(result.objectOrigin))
        result.objectOrigin.computeAcceleration = true
    end
    if (!isnothing(result.objectObserveRef))
        result.objectObserveRef.computeAcceleration = true
    end

    result.translationResultIndex          = Modia.new_w_segmented_variable!(model, result.path*".translation"          , SVector{3,F}(0, 0, 0), "m")
    result.velocityResultIndex             = Modia.new_w_segmented_variable!(model, result.path*".velocity"             , SVector{3,F}(0, 0, 0), "m/s")
    result.accelerationResultIndex         = Modia.new_w_segmented_variable!(model, result.path*".acceleration"         , SVector{3,F}(0, 0, 0), "m/s^2")
    result.rotationResultIndex             = Modia.new_w_segmented_variable!(model, result.path*".rotation"             , SVector{3,F}(0, 0, 0), "rad")
    result.angularVelocityResultIndex      = Modia.new_w_segmented_variable!(model, result.path*".angularVelocity"      , SVector{3,F}(0, 0, 0), "rad/s")
    result.angularAccelerationResultIndex  = Modia.new_w_segmented_variable!(model, result.path*".angularAcceleration"  , SVector{3,F}(0, 0, 0), "rad/s^2")
    result.distanceResultIndex             = Modia.new_w_segmented_variable!(model, result.path*".distance"             , F(0)                 , "m")
    result.distanceVelocityResultIndex     = Modia.new_w_segmented_variable!(model, result.path*".distanceVelocity"     , F(0)                 , "m/s")
    result.distanceAccelerationResultIndex = Modia.new_w_segmented_variable!(model, result.path*".distanceAcceleration" , F(0)                 , "m/s^2")

    return nothing
end

function evaluateResultElement(model::Modia.InstantiatedModel{F,TimeType}, scene::Modia3D.Composition.Scene{F}, result::SensorResult{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    translation = measFramePosition(result.object; frameOrig=result.objectOrigin, frameCoord=result.objectCoordinateRef)
    velocity = measFrameTransVelocity(result.object; frameOrig=result.objectOrigin, frameCoord=result.objectCoordinateRef, frameObsrv=result.objectObserveRef)
    acceleration = measFrameTransAcceleration(result.object; frameOrig=result.objectOrigin, frameCoord=result.objectCoordinateRef, frameObsrv=result.objectObserveRef)
    rotationMatrix = measFrameRotation(result.object; frameOrig=result.objectOrigin)
    rotation = rot123fromR(rotationMatrix)
    angularVelocity = measFrameRotVelocity(result.object; frameOrig=result.objectOrigin, frameCoord=result.objectCoordinateRef)
    angularAcceleration = measFrameRotAcceleration(result.object; frameOrig=result.objectOrigin, frameCoord=result.objectCoordinateRef, frameObsrv=result.objectObserveRef)
    (distance, dir) = measFrameDistance(result.object; frameOrig=result.objectOrigin)
    distanceVelocity = measFrameDistVelocity(result.object; frameOrig=result.objectOrigin)
    distanceAcceleration = measFrameDistAcceleration(result.object; frameOrig=result.objectOrigin)

    Modia.copy_w_segmented_value_to_result(model, result.translationResultIndex, translation)
    Modia.copy_w_segmented_value_to_result(model, result.velocityResultIndex, velocity)
    Modia.copy_w_segmented_value_to_result(model, result.accelerationResultIndex, acceleration)
    Modia.copy_w_segmented_value_to_result(model, result.rotationResultIndex, rotation)
    Modia.copy_w_segmented_value_to_result(model, result.angularVelocityResultIndex, angularVelocity)
    Modia.copy_w_segmented_value_to_result(model, result.angularAccelerationResultIndex, angularAcceleration)
    Modia.copy_w_segmented_value_to_result(model, result.distanceResultIndex, distance)
    Modia.copy_w_segmented_value_to_result(model, result.distanceVelocityResultIndex, distanceVelocity)
    Modia.copy_w_segmented_value_to_result(model, result.distanceAccelerationResultIndex, distanceAcceleration)

    return nothing
end

function terminateResultElement(result::SensorResult{F}) where F <: Modia3D.VarFloatType
    return nothing
end
