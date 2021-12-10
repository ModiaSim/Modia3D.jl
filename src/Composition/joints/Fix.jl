# License for this file: MIT (expat)
# Copyright 2017-2021, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

"""
    obj = Fix(; obj1, obj2, translation=zeros(3), rotation=zeros(3))

Rigidly fix `obj2::`[`Object3D`](@ref) relatively to
`obj1::`[`Object3D`](@ref) via the relative position vector `translation` from
`obj1` to `obj2` (resolved in `obj1`) and the relative orientation angles
`rotation` from `obj1` to `obj2`.

Return `obj=obj1`, if `obj2` is directly or indirectly
connected to the `world` Object3D. Otherwise, return `obj=obj2`.

# Arguments
- `obj1::Object3D`: Parent object.

- `obj2::Object3D`: Object fixed relative to `obj1`.

- `r::AbstractVector`: Relative position vector from `obj1` to `obj2`
   resolved in `obj1`.

- `rot::AbstractVector`: Relative orientation from `obj1` to `obj2`
   in [Cardan (Tait–Bryan) angles](https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence)
   (rotation sequence x-y-z).
"""
function Fix(; obj1::Object3D,
               obj2::Object3D,
               translation::AbstractVector = Modia3D.ZeroVector3D(Float64),
               rotation::AbstractVector    = Modia3D.ZeroVector3D(Float64))::Object3D

    (parent, child, cutJoint) = attach(obj1, obj2)
    if cutJoint
        error("\nError from Fix joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":",
              "\nThis joint is a cut-joint which is not allowed.")
    end

    r_rel = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"m", translation)
    rot   = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"rad", rotation)

    if obj1 === parent
        # obj1/obj2 = parent/child
        obj2.r_rel      = r_rel
        obj2.jointIndex = 0
        obj2.ndof       = 0

        if rot == Modia3D.ZeroVector3D(Float64)
            obj2.joint     = FixedTranslationJoint()
            obj2.jointKind = FixTranslationKind
            obj2.R_rel     = Frames.NullRotation
            obj2.R_abs     = obj1.R_abs
            obj2.r_abs     = obj1.r_abs + r_rel
        else
            obj2.joint     = FixedJoint()
            obj2.jointKind = FixKind
            obj2.R_rel     = Frames.rot123(rot[1], rot[2], rot[3])
            obj2.R_abs     = obj2.R_rel*obj1.R_abs
            obj2.r_abs     = obj1.r_abs + obj2.R_abs'*r_rel
        end
        return obj2

    else
        # obj1/obj2 = child/parent
        obj1.jointIndex = 0
        obj1.ndof       = 0

        if rot == Modia3D.ZeroVector3D(Float64)
            obj1.joint     = FixedTranslationJoint()
            obj1.jointKind = FixTranslationKind
            obj1.R_rel     = Frames.NullRotation
            obj1.R_abs     = obj2.R_abs
            obj1.r_rel     = -r_rel
        else
            obj1.joint     = FixedJoint()
            obj1.jointKind = FixKind
            obj1.R_rel     = Frames.rot123(rot[1], rot[2], rot[3])'
            obj1.R_abs     = obj1.R_rel*obj2.R_abs
            obj1.r_rel     = -obj1.R_rel'*r_rel
            obj1.r_abs     = obj2.r_abs - obj2.R_abs*r_rel
        end
        return obj1
    end
end
