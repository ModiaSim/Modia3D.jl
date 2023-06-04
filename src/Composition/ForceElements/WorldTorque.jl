
"""
    torque = WorldTorque(; objectApply,
                           torqueFunction,
                           objectCoord = nothing)

Return a `torque` acting at `objectApply::`[`Object3D`](@ref)
calculated by function `torqueFunction`. The torque directions are
defined by the coordinate axes of `objectCoord::`[`Object3D`](@ref).

# Arguments

- `torqueFunction` defines the torque vector calculation function:
  `torqueVector = torqueFunction(; time::TimeType, objectApply::Object3D{FloatType}, objectCoord::Object3D{FloatType})::SVector{3,FloatType} where {TimeType, FloatType}`
- `objectCoord` defines an [`Object3D`](@ref) which defines the
  coordinate directions of the torque vector. In case of
  `objectCoord = nothing` the torque vector is applied in World
  coordinates.

# Results

- `torqueVector` is the torque vector acting on `objectApply`,
  resolved in `objectCoord`.
"""
mutable struct WorldTorque{F <: Modia3D.VarFloatType} <: Modia3D.AbstractForceElement

    path::String

    objectApply::Object3D{F}
    objectCoord::Union{Object3D{F}, Nothing}

    torqueFunction::Function

    torqueVectorResultIndex::Int

    function WorldTorque{F}(; path::String = "",
                              objectApply::Object3D{F},
                              torqueFunction::Function,
                              objectCoord::Union{Object3D{F}, Nothing} = nothing) where F <: Modia3D.VarFloatType

        return new(path, objectApply, objectCoord, torqueFunction)
    end
end
WorldTorque(; kwargs...) = WorldTorque{Float64}(; kwargs...)


function initializeForceElement(model::Modia.InstantiatedModel{F,TimeType}, force::WorldTorque{F})::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    force.objectApply.hasForceElement = true
    if !isnothing(force.objectCoord)
        force.objectCoord.hasForceElement = true
    end

    force.torqueVectorResultIndex = Modia.new_w_segmented_variable!(model, force.path*".torqueVector", SVector{3,F}(0, 0, 0), "N*m")

    return nothing
end

function evaluateForceElement(model::Modia.InstantiatedModel{F,TimeType}, force::WorldTorque{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    torqueVector = force.torqueFunction(; time=time, objectApply=force.objectApply, objectCoord=force.objectCoord)

    applyFrameTorque!(force.objectApply, torqueVector; frameCoord=force.objectCoord)

    if Modia.storeResults(model)
        Modia.copy_w_segmented_value_to_result(model, force.torqueVectorResultIndex, torqueVector)
    end

    return nothing
end

function terminateForceElement(force::WorldTorque{F})::Nothing where F <: Modia3D.VarFloatType
    return nothing
end
