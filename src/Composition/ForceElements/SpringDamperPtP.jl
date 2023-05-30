
"""
    force = SpringDamperPtP(; obj1, obj2,
        nominalLength  = 0.0,
        nominalForce   = 0.0,
        springForceLaw = 0.0,
        damperForceLaw = 0.0 )

Return a `force` acting as point-to-point parallel spring-damper between
`obj1::`[`Object3D`](@ref) and `obj2::`[`Object3D`](@ref).

# Arguments

- `nominalLength` defines the nominal length, i.e. the distance between
  `obj1` and `obj2` where the deflection of the spring is zero.
- `nominalForce` defines the nominal force, i.e. the force that acts when
  spring and damper forces are zero. Positive values represent tension.
- `springForceLaw` defines the force law of the spring:
  - A `Real` number represents a linear stiffness coefficient.
  - An univariate `Function` is used to compute the spring force
    dependent of its deflection. Positive values represent tension.
- `damperForceLaw` defines the force law of the damper:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the damper force
    dependent of its deflection velocity. Positive values represent
    expansion.

# Results

- `length` is the distance between `obj1` and `obj2`.
- `deflection` is the spring deflection.
- `velocity` is the point-to-point velocity between `obj1` and `obj2`.
- `springForce` is the spring force (including the nominal force).
- `damperForce` is the damper force.
- `force` is the total force.
- `forceVector` is the force vector acting on `obj2`, resolved in `obj1`.
   On `obj1` the same force vector is applied in inverse direction.
"""
mutable struct SpringDamperPtP{F <: Modia3D.VarFloatType} <: Modia3D.AbstractForceElement

    path::String

    obj1::Object3D{F}
    obj2::Object3D{F}

    nominalLength::F
    nominalForce::F
    springForceFunction::Function
    damperForceFunction::Function

    distanceResultIndex::Int
    deflectionResultIndex::Int
    velocityResultIndex::Int
    springForceResultIndex::Int
    damperForceResultIndex::Int
    forceResultIndex::Int
    forceVectorResultIndex::Int

    function SpringDamperPtP{F}(; path::String = "",
                                  obj1::Object3D{F},
                                  obj2::Object3D{F},
                                  nominalLength::Real = F(0.0),
                                  nominalForce::Real = F(0.0),
                                  springForceLaw::Union{Real, Function} = F(0.0),
                                  damperForceLaw::Union{Real, Function} = F(0.0) ) where F <: Modia3D.VarFloatType

        nomLength = Modia3D.convertAndStripUnit(F, u"m", nominalLength)
        nomForce  = Modia3D.convertAndStripUnit(F, u"N", nominalForce)
        if (!isa(springForceLaw, Function))
            stiffness = Modia3D.convertAndStripUnit(F, u"N/m", springForceLaw)
            fsymb = Symbol(path, "_", "fc")
            springForceLaw = eval(:($fsymb(pos) = $stiffness * pos))
        end
        if (!isa(damperForceLaw, Function))
            damping = Modia3D.convertAndStripUnit(F, u"N*s/m", damperForceLaw)
            fsymb = Symbol(path, "_", "fd")
            damperForceLaw = eval(:($fsymb(vel) = $damping * vel))
        end

        return new(path, obj1, obj2, nomLength, nomForce, springForceLaw, damperForceLaw)
    end
end
SpringDamperPtP(; kwargs...) = SpringDamperPtP{Float64}(; kwargs...)


function initializeForceElement(model::Modia.SimulationModel{F,TimeType}, force::SpringDamperPtP{F}) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true

    force.distanceResultIndex    = Modia.new_w_segmented_variable!(model, force.path*".distance"   , F(0), "m")
    force.deflectionResultIndex  = Modia.new_w_segmented_variable!(model, force.path*".deflection" , F(0), "m")
    force.velocityResultIndex    = Modia.new_w_segmented_variable!(model, force.path*".velocity"   , F(0), "m/s")
    force.springForceResultIndex = Modia.new_w_segmented_variable!(model, force.path*".springForce", F(0), "N")
    force.damperForceResultIndex = Modia.new_w_segmented_variable!(model, force.path*".damperForce", F(0), "N")
    force.forceResultIndex       = Modia.new_w_segmented_variable!(model, force.path*".force"      , F(0), "N")
    force.forceVectorResultIndex = Modia.new_w_segmented_variable!(model, force.path*".forceVector", SVector{3,F}(0, 0, 0), "N")

    return nothing
end

function evaluateForceElement(model::Modia.SimulationModel{F,TimeType}, force::SpringDamperPtP{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    (pos, norm) = measFrameDistance(force.obj2; frameOrig=force.obj1)
    vel = measFrameDistVelocity(force.obj2; frameOrig=force.obj1)

    defl = pos - force.nominalLength
    fc = force.springForceFunction(defl) + force.nominalForce
    fd = force.damperForceFunction(vel)
    frc = fc + fd
    f12 = frc * norm

    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)

    if Modia.storeResults(model)
        Modia.add_w_segmented_value!(model, force.distanceResultIndex, pos)
        Modia.add_w_segmented_value!(model, force.deflectionResultIndex, defl)
        Modia.add_w_segmented_value!(model, force.velocityResultIndex, vel)
        Modia.add_w_segmented_value!(model, force.springForceResultIndex, fc)
        Modia.add_w_segmented_value!(model, force.damperForceResultIndex, fd)
        Modia.add_w_segmented_value!(model, force.forceResultIndex, frc)
        Modia.add_w_segmented_value!(model, force.forceVectorResultIndex, -f12)
    end

    return nothing
end

function terminateForceElement(force::SpringDamperPtP{F}) where F <: Modia3D.VarFloatType
    return nothing
end
