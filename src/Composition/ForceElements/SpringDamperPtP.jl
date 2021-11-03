
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
  A `Float64` value represents a linear stiffness coefficient.
  An univariate `Function` is used to compute the spring force dependent
  of its deflection. Positive values represent tension.
- `damperForceLaw` defines the force law of the damper:
  A `Float64` value represents a linear damping coefficient.
  An univariate `Function` is used to compute the damper force dependent
  of its deflection velocity. Positive values represent expansion.
"""
mutable struct SpringDamperPtP <: Modia3D.AbstractForceElement

    obj1::Object3D
    obj2::Object3D

    nominalLength::Float64
    nominalForce::Float64
    springForceFunction::Function
    damperForceFunction::Function

    function SpringDamperPtP(; obj1::Object3D,
                               obj2::Object3D,
                               nominalLength::Float64 = 0.0,
                               nominalForce::Float64 = 0.0,
                               springForceLaw::Union{Float64, Function} = 0.0,
                               damperForceLaw::Union{Float64, Function} = 0.0 )

        nomLength = Modia3D.convertAndStripUnit(Float64, u"m", nominalLength)
        nomForce  = Modia3D.convertAndStripUnit(Float64, u"N", nominalForce)
        if (typeof(springForceLaw) == Float64)
            stiffness = Modia3D.convertAndStripUnit(Float64, u"N/m", springForceLaw)
            springForceLaw = eval(:(fc(pos) = $stiffness * pos))
        end
        if (typeof(damperForceLaw) == Float64)
            damping = Modia3D.convertAndStripUnit(Float64, u"N*s/m", damperForceLaw)
            damperForceLaw = eval(:(fd(vel) = $damping * vel))
        end

        return new(obj1, obj2, nomLength, nomForce, springForceLaw, damperForceLaw)
    end
end


function initializeForceElement(force::SpringDamperPtP)
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true
    return nothing
end

function evaluateForceElement(force::SpringDamperPtP)
    (pos, norm) = measFrameDistance(force.obj2; frameOrig=force.obj1)
    vel = measFrameDistVelocity(force.obj2; frameOrig=force.obj1)

    defl = pos - force.nominalLength
    fc = force.springForceFunction(defl)
    fd = force.damperForceFunction(vel)
    f12 = (fc + fd + force.nominalForce) * norm

    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
end

function terminateForceElement(force::SpringDamperPtP)
    return nothing
end
