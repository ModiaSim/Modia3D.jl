# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

"""
    joint = Prismatic(; obj1, obj2, axis=1, s=0, v=0, canCollide=true)

Return a Prismatic `joint` that translates `obj2::`[`Object3D`](@ref) with respect to
`obj1::`[`Object3D`](@ref) along coordinate axis `axis` (`axis = 1,2,3,-1,-2,-3`) 
of `obj1` with signed distance `s`. Optionally, `axis` can be a vector, such as `axis = [1.0, 2.0, 3.0]`, 
defining the direction of the axis of translation in `obj1`. The initial position is `s` 
and the initial velocity is `v`.

If `canCollide=false`, no collision detection will occur between `obj1` and `obj2` (and `Object3D`s that are
directly or indirectly rigidly fixed to `obj1` or `obj2`). Note, if both `obj1` and `obj2` have
solids defined that are overlapping along the axis of translation, then `canCollide=false`
is the only meaningful value.

It is currently not supported that a `Prismatic` joint *closes a kinematic loop*.
"""
mutable struct Prismatic{F <: Modia3D.VarFloatType} <: Modia3D.AbstractJoint
    path::String

    obj1::Object3D{F}
    obj2::Object3D{F}

    eAxis::SVector{3,F}   # Translation axis with norm(eAxis) = 1
    canCollide::Bool      # = false, if no collision between obj1 and obj2

    s::F
    v::F
    a::F

    function Prismatic{F}(; obj1::Object3D{F},
                            obj2::Object3D{F},
                            path::String="",
                            axis::Union{Int, AbstractVector} = 1,
                            s::Real = F(0.0),
                            v::Real = F(0.0),
                            canCollide::Bool = true) where F <: Modia3D.VarFloatType

        (parent,obj,cutJoint) = attach(obj1, obj2, name = "Prismatic joint")  # an error is triggered if cutJoint=true

        if typeof(axis) <: Int
            eAxis =  abs(axis) == 1 ? SVector{3,F}(1, 0, 0) :
                    (abs(axis) == 2 ? SVector{3,F}(0, 1, 0) :
                    (abs(axis) == 3 ? SVector{3,F}(0, 0, 1) :
                        error("\nError from $path = Prismatic(obj1 = :($(obj1.path)), obj2 = :($(obj2.path)), axis = $axis, ...):\n",
                              "    axis = must be 1,2,3 or -1,-2,-3 or a vector with 3 elements!")))
            if axis < 0
                eAxis = -eAxis
            end
        else
            eAxis = normalize(SVector{3,F}(ustrip.(axis)))
            if !all(isfinite,eAxis)
                error("\nError from $path = Prismatic(obj1 = :($(obj1.path)), obj2 = :($(obj2.path)), axis = $axis, ...):\n",
                      "    normalize(axis) results in vector $eAxis that has non-finite elements!")
            end
        end

        eAxis = obj === obj2 ? eAxis : -eAxis
        s = Modia3D.convertAndStripUnit(F, u"m"  , s)
        v = Modia3D.convertAndStripUnit(F, u"m/s", v)

        obj.joint      = new(path, parent, obj, eAxis, canCollide, s, v, F(0.0) )
        obj.jointKind  = PrismaticKind
        obj.jointIndex = 0
        obj.ndof       = 1
        obj.canCollide = canCollide
        obj.r_rel      = eAxis*s
        obj.R_rel      = Modia3D.NullRotation(F)
        parent.hasChildJoint = true
        return obj.joint
    end
end
