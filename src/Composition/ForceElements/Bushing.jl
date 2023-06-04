
"""
    force = Bushing(; obj1, obj2,
        nominalForce      = [0.0, 0.0, 0.0],
        springForceLaw    = [0.0, 0.0, 0.0],
        damperForceLaw    = [0.0, 0.0, 0.0],
        nominalTorque     = [0.0, 0.0, 0.0],
        rotSpringForceLaw = [0.0, 0.0, 0.0],
        rotDamperForceLaw = [0.0, 0.0, 0.0],
        largeAngles       = false )

Return a `force` acting as bushing between `obj1::`[`Object3D`](@ref) and
`obj2::`[`Object3D`](@ref). The force directions are defined by `obj1`,
i.e. the orientation of `obj2` does not influence the resulting forces.

# Arguments

- `nominalForce` defines the nominal force vector, i.e. the force that
  acts when spring and damper forces are zero. Positive values act in
  positive axis directions at `obj1` and in opposite directions at `obj2`.
- `springForceLaw` defines the force law of the spring in x-, y- and
  z-direction:
  - A `Real` number represents a linear stiffness coefficient.
  - An univariate `Function` is used to compute the spring force
    dependent of its deflection.
- `damperForceLaw` defines the force law of the damper in x-, y- and
  z-direction:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the damper force
    dependent of its deflection velocity.
- `nominalTorque` defines nominal torques about alpha, beta and gamma
  directions.
- `rotSpringForceLaw` defines the force law of the rotational spring
  about alpha-, beta- and gamma-direction:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the spring force
    dependent of its deflection.
- `rotDamperForceLaw` defines the force law of the rotational damper
  about alpha-, beta- and gamma-direction:
  - A `Real` number represents a linear damping coefficient.
  - An univariate `Function` is used to compute the damper force
    dependent of its deflection velocity.
- `largeAngles` can be used to enable large angle mode.
  - When disabled, small deformation angles (< 10°) are assumed. This
    option deals equally with rotations [alpha, beta gamma] about the
    axes [x, y, z] of `obj1`, but causes approximation errors for
    larger angles.
  - When enabled, the deformation angles and torque directions are
    calculated as [Cardan (Tait–Bryan) angles](https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence)
    (rotation sequence x-y-z from `obj1` to `obj2`). This option
    supports angles up to nearly 90°, but introduces local rotation
    directions [alpha, beta gamma] which differ from the axes [x, y, z]
    of `obj1` and increases computation effort.

# Results

- `translation` is the translation vector from `obj1` to `obj2`,
  resolved in `obj1`.
- `rotation` contains the rotation angles alpha, beta and gamma from
  `obj1` to `obj2`.
- `velocity` is the translation velocity vector from `obj1` to `obj2`,
  resolved in `obj1`.
- `rotationVelocity` contains the rotation angular velocities about
  alpha-, beta- and gamma-direction from `obj1` to `obj2`.
- `springForce` is the spring force vector.
- `springTorque` contains the spring torques.
- `damperForce` is the damper force vector.
- `damperTorque` contains the damper torques.
- `torque` contains the total torques.
- `forceVector` is the force vector acting on `obj2`, resolved in `obj1`.
   At `obj1` the same force vector is applied in inverse direction. In
   addition a compensation torque is applied at `obj1` to satisfy torque
   balance.
- `torqueVector` is the torque vector acting on `obj2`, resolved in
  `obj1`. At `obj1` the same torque vector is applied in inverse
  direction.
"""
mutable struct Bushing{F <: Modia3D.VarFloatType} <: Modia3D.AbstractForceElement

    path::String

    obj1::Object3D{F}
    obj2::Object3D{F}

    nominalForce::SVector{3,F}
    springForceFunction::SVector{3,Function}
    damperForceFunction::SVector{3,Function}
    nominalTorque::SVector{3,F}
    rotSpringForceFunction::SVector{3,Function}
    rotDamperForceFunction::SVector{3,Function}
    largeAngles::Bool

    translationResultIndex::Int
    rotationResultIndex::Int
    velocityResultIndex::Int
    rotationVelocityResultIndex::Int
    springForceResultIndex::Int
    springTorqueResultIndex::Int
    damperForceResultIndex::Int
    damperTorqueResultIndex::Int
    torqueResultIndex::Int
    forceVectorResultIndex::Int
    torqueVectorResultIndex::Int

    function Bushing{F}(; path::String = "",
                          obj1::Object3D{F},
                          obj2::Object3D{F},
                          nominalForce::AbstractVector = Modia3D.ZeroVector3D(F),
                          springForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          damperForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          nominalTorque::AbstractVector = Modia3D.ZeroVector3D(F),
                          rotSpringForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          rotDamperForceLaw::AbstractVector = Modia3D.ZeroVector3D(F),
                          largeAngles::Bool = false ) where F <: Modia3D.VarFloatType
        nomForce  = Modia3D.convertAndStripUnit(SVector{3,F}, u"N"  , nominalForce)
        nomTorque = Modia3D.convertAndStripUnit(SVector{3,F}, u"N*m", nominalTorque)
        springForceFunction = Vector{Function}(undef, 3)
        damperForceFunction = Vector{Function}(undef, 3)
        rotSpringForceFunction = Vector{Function}(undef, 3)
        rotDamperForceFunction = Vector{Function}(undef, 3)
        for dir in 1:3
            if (isa(springForceLaw[dir], Function))
                springForceFunction[dir] = springForceLaw[dir]
            else
                stiffness = Modia3D.convertAndStripUnit(F, u"N/m", springForceLaw[dir])
                fsymb = Symbol(path, "_", "fc", dir)
                springForceFunction[dir] = eval(:($fsymb(pos) = $stiffness * pos))
            end
            if (isa(damperForceLaw[dir], Function))
                damperForceFunction[dir] = damperForceLaw[dir]
            else
                damping = Modia3D.convertAndStripUnit(F, u"N*s/m", damperForceLaw[dir])
                fsymb = Symbol(path, "_", "fd", dir)
                damperForceFunction[dir] = eval(:($fsymb(vel) = $damping * vel))
            end
            if (isa(rotSpringForceLaw[dir], Function))
                rotSpringForceFunction[dir] = rotSpringForceLaw[dir]
            else
                stiffness = Modia3D.convertAndStripUnit(F, u"N*m/rad", rotSpringForceLaw[dir])
                fsymb = Symbol(path, "_", "mc", dir)
                rotSpringForceFunction[dir] = eval(:($fsymb(ang) = $stiffness * ang))
            end
            if (isa(rotDamperForceLaw[dir], Function))
                rotDamperForceFunction[dir] = rotDamperForceLaw[dir]
            else
                damping = Modia3D.convertAndStripUnit(F, u"N*m*s/rad", rotDamperForceLaw[dir])
                fsymb = Symbol(path, "_", "md", dir)
                rotDamperForceFunction[dir] = eval(:($fsymb(angd) = $damping * angd))
            end
        end

        return new(path, obj1, obj2, nomForce, springForceFunction, damperForceFunction, nomTorque, rotSpringForceFunction, rotDamperForceFunction, largeAngles)
    end
end
Bushing(; kwargs...) = Bushing{Float64}(; kwargs...)


# Compute deformation angles from rotation matrix
function anglesFromRotation(largeAngles::Bool, R12::SMatrix{3,3,F,9}, w12::SVector{3,F})::Tuple{SVector{3,F},SVector{3,F},SMatrix{2,2,F,4}} where F <: Modia3D.VarFloatType
    if largeAngles
        sbe = clamp(R12[3,1], F(-1.0), F(1.0))
        cbe = sqrt(F(1.0) - sbe*sbe)
        if (cbe > 1e-12)
            sal = -R12[3,2]/cbe
            cal =  R12[3,3]/cbe
            al = atan(-R12[3,2], R12[3,3])
            be = asin(sbe)
            ga = atan(-R12[2,1], R12[1,1])
            ald = w12[1] + (sal*w12[2] - cal*w12[3])*sbe/cbe
            bed = cal*w12[2] + sal*w12[3]
            gad = (-sal*w12[2] + cal*w12[3])/cbe
            return (SVector{3,F}(al, be, ga), SVector{3,F}(ald, bed, gad), SMatrix{2,2,F,4}(sal, cal, sbe, cbe))
        else
            @error("Gimbal lock of Bushing transformation.")
            return (SVector{3,F}(0.0, 0.0, 0.0), SVector{3,F}(0.0, 0.0, 0.0), SMatrix{2,2,F,4}(0.0, 0.0, 0.0, 0.0))
        end
    else
        al = R12[2,3]
        be = R12[3,1]
        ga = R12[1,2]
        ald = w12[1]
        bed = w12[2]
        gad = w12[3]
        if (max(abs(al), abs(be), abs(ga)) > 0.174)
            @warn("Bushing angle exceeds 10 deg.")
        end
        return (SVector{3,F}(al, be, ga), SVector{3,F}(ald, bed, gad), SMatrix{2,2,F,4}(0.0, 0.0, 0.0, 0.0))
    end
end

# Compute torque vector from force law moments
function torqueFromMoments(largeAngles::Bool, moments::SVector{3,F}, sico::SMatrix{2,2,F,4})::SVector{3,F} where F <: Modia3D.VarFloatType
    if largeAngles
        tx = moments[1] + sico[1,2]*moments[3]
        ty = sico[2,1]*moments[2] - sico[1,1]*sico[2,2]*moments[3]
        tz = sico[1,1]*moments[2] + sico[2,1]*sico[2,2]*moments[3]
        return SVector{3,F}(tx, ty, tz)
    else
        return moments
    end
end


function initializeForceElement(model::Modia.SimulationModel{F,TimeType}, force::Bushing{F})::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true

    force.translationResultIndex      = Modia.new_w_segmented_variable!(model, force.path*".translation"     , SVector{3,F}(0, 0, 0), "m")
    force.rotationResultIndex         = Modia.new_w_segmented_variable!(model, force.path*".rotation"        , SVector{3,F}(0, 0, 0), "rad")
    force.velocityResultIndex         = Modia.new_w_segmented_variable!(model, force.path*".velocity"        , SVector{3,F}(0, 0, 0), "m/s")
    force.rotationVelocityResultIndex = Modia.new_w_segmented_variable!(model, force.path*".rotationVelocity", SVector{3,F}(0, 0, 0), "rad/s")
    force.springForceResultIndex      = Modia.new_w_segmented_variable!(model, force.path*".springForce"     , SVector{3,F}(0, 0, 0), "N")
    force.springTorqueResultIndex     = Modia.new_w_segmented_variable!(model, force.path*".springTorque"    , SVector{3,F}(0, 0, 0), "N*m")
    force.damperForceResultIndex      = Modia.new_w_segmented_variable!(model, force.path*".damperForce"     , SVector{3,F}(0, 0, 0), "N")
    force.damperTorqueResultIndex     = Modia.new_w_segmented_variable!(model, force.path*".damperTorque"    , SVector{3,F}(0, 0, 0), "N*m")
    force.torqueResultIndex           = Modia.new_w_segmented_variable!(model, force.path*".torque"          , SVector{3,F}(0, 0, 0), "N*m")
    force.forceVectorResultIndex      = Modia.new_w_segmented_variable!(model, force.path*".forceVector"     , SVector{3,F}(0, 0, 0), "N")
    force.torqueVectorResultIndex     = Modia.new_w_segmented_variable!(model, force.path*".torqueVector"    , SVector{3,F}(0, 0, 0), "N*m")

    return nothing
end

function evaluateForceElement(model::Modia.SimulationModel{F,TimeType}, force::Bushing{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    R12 = measFrameRotation(force.obj2; frameOrig=force.obj1)
    r12 = measFramePosition(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    w12 = measFrameRotVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    v12 = measFrameTransVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1, frameObsrv=force.obj1)
    (ang, angd, sico) = anglesFromRotation(force.largeAngles, R12, w12)

    fc = zeros(SVector{3,F})
    fd = zeros(SVector{3,F})
    mc = zeros(SVector{3,F})
    md = zeros(SVector{3,F})
    for dir in 1:3
        fc = setindex(fc, force.springForceFunction[dir](r12[dir]), dir)
        fd = setindex(fd, force.damperForceFunction[dir](v12[dir]), dir)
        mc = setindex(mc, force.rotSpringForceFunction[dir](ang[dir]), dir)
        md = setindex(md, force.rotDamperForceFunction[dir](angd[dir]), dir)
    end
    f12 = fc + fd + force.nominalForce
    mom = mc + md + force.nominalTorque
    t12 = torqueFromMoments(force.largeAngles, mom, sico)

    applyFrameForcePair!(force.obj2, force.obj1, f12; frameCoord=force.obj1)
    applyFrameTorquePair!(force.obj2, force.obj1, t12; frameCoord=force.obj1)

    if Modia.storeResults(model)
        Modia.copy_w_segmented_value_to_result(model, force.translationResultIndex, r12)
        Modia.copy_w_segmented_value_to_result(model, force.rotationResultIndex, ang)
        Modia.copy_w_segmented_value_to_result(model, force.velocityResultIndex, v12)
        Modia.copy_w_segmented_value_to_result(model, force.rotationVelocityResultIndex, angd)
        Modia.copy_w_segmented_value_to_result(model, force.springForceResultIndex, fc)
        Modia.copy_w_segmented_value_to_result(model, force.springTorqueResultIndex, mc)
        Modia.copy_w_segmented_value_to_result(model, force.damperForceResultIndex, fd)
        Modia.copy_w_segmented_value_to_result(model, force.damperTorqueResultIndex, md)
        Modia.copy_w_segmented_value_to_result(model, force.torqueResultIndex, mom)
        Modia.copy_w_segmented_value_to_result(model, force.forceVectorResultIndex, -f12)
        Modia.copy_w_segmented_value_to_result(model, force.torqueVectorResultIndex, -t12)
    end

    return nothing
end

function terminateForceElement(force::Bushing{F})::Nothing where F <: Modia3D.VarFloatType
    return nothing
end
