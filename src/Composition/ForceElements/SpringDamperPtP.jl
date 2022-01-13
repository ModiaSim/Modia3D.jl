
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
"""
mutable struct SpringDamperPtP{F <: Modia3D.VarFloatType} <: Modia3D.AbstractForceElement

    obj1::Object3D{F}
    obj2::Object3D{F}

    nominalLength::F
    nominalForce::F
    springForceFunction::Function
    damperForceFunction::Function

    function SpringDamperPtP{F}(; obj1::Object3D{F},
                                  obj2::Object3D{F},
                                  nominalLength::Real = F(0.0),
                                  nominalForce::Real = F(0.0),
                                  springForceLaw::Union{Real, Function} = F(0.0),
                                  damperForceLaw::Union{Real, Function} = F(0.0) ) where F <: Modia3D.VarFloatType

        nomLength = Modia3D.convertAndStripUnit(F, u"m", nominalLength)
        nomForce  = Modia3D.convertAndStripUnit(F, u"N", nominalForce)
        irand = rand(Int)
        if (!isa(springForceLaw, Function))
            stiffness = Modia3D.convertAndStripUnit(F, u"N/m", springForceLaw)
            fsymb = Symbol("fc", "_", irand)  # todo: replace irand by force.path
            springForceLaw = eval(:($fsymb(pos) = $stiffness * pos))
        end
        if (!isa(damperForceLaw, Function))
            damping = Modia3D.convertAndStripUnit(F, u"N*s/m", damperForceLaw)
            fsymb = Symbol("fd", "_", irand)  # todo: replace irand by force.path
            damperForceLaw = eval(:(fd(vel) = $damping * vel))
        end

        return new(obj1, obj2, nomLength, nomForce, springForceLaw, damperForceLaw)
    end
end
SpringDamperPtP(; kwargs...) = SpringDamperPtP{Float64}(; kwargs...)


function initializeForceElement(force::SpringDamperPtP{F}) where F <: Modia3D.VarFloatType
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true
    return nothing
end

function evaluateForceElement(force::SpringDamperPtP{F}) where F <: Modia3D.VarFloatType
    (pos, norm) = measFrameDistance(force.obj2; frameOrig=force.obj1)
    vel = measFrameDistVelocity(force.obj2; frameOrig=force.obj1)

    defl = pos - force.nominalLength
    fc = force.springForceFunction(defl)
    fd = force.damperForceFunction(vel)
    f12 = (fc + fd + force.nominalForce) * norm

    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
end

function terminateForceElement(force::SpringDamperPtP{F}) where F <: Modia3D.VarFloatType
    return nothing
end
