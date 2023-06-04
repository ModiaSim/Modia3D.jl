
"""
    force = WorldForce(; objectApply,
                         forceFunction,
                         objectCoord = nothing)

Return a `force` acting at `objectApply::`[`Object3D`](@ref)
calculated by function `forceFunction`. The force directions are
defined by the coordinate axes of `objectCoord::`[`Object3D`](@ref).

# Arguments

- `forceFunction` defines the force vector calculation function:
  `forceVector = forceFunction(; time::TimeType, objectApply::Object3D{FloatType}, objectCoord::Object3D{FloatType})::SVector{3,FloatType} where {TimeType, FloatType}`
- `objectCoord` defines an [`Object3D`](@ref) which defines the
  coordinate directions of the force vector. In case of
  `objectCoord = nothing` the force vector is applied in World
  coordinates.

# Results

- `forceVector` is the force vector acting on `objectApply`, resolved
  in `objectCoord`.
"""
mutable struct WorldForce{F <: Modia3D.VarFloatType} <: Modia3D.AbstractForceElement

    path::String

    objectApply::Object3D{F}
    objectCoord::Union{Object3D{F}, Nothing}

    forceFunction::Function

    forceVectorResultIndex::Int

    function WorldForce{F}(; path::String = "",
                             objectApply::Object3D{F},
                             forceFunction::Function,
                             objectCoord::Union{Object3D{F}, Nothing} = nothing) where F <: Modia3D.VarFloatType

        return new(path, objectApply, objectCoord, forceFunction)
    end
end
WorldForce(; kwargs...) = WorldForce{Float64}(; kwargs...)


function initializeForceElement(model::Modia.SimulationModel{F,TimeType}, force::WorldForce{F})::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    force.objectApply.hasForceElement = true
    if !isnothing(force.objectCoord)
        force.objectCoord.hasForceElement = true
    end

    force.forceVectorResultIndex = Modia.new_w_segmented_variable!(model, force.path*".forceVector", SVector{3,F}(0, 0, 0), "N")

    return nothing
end

function evaluateForceElement(model::Modia.SimulationModel{F,TimeType}, force::WorldForce{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    forceVector = force.forceFunction(; time=time, objectApply=force.objectApply, objectCoord=force.objectCoord)

    applyFrameForce!(force.objectApply, forceVector; frameCoord=force.objectCoord)

    if Modia.storeResults(model)
        Modia.copy_w_segmented_value_to_result(model, force.forceVectorResultIndex, forceVector)
    end

    return nothing
end

function terminateForceElement(force::WorldForce{F})::Nothing where F <: Modia3D.VarFloatType
    return nothing
end
