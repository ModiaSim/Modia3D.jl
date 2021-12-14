@enum RotationVariables               RCardan123
@enum WStartVariables    WCartesian   WCardan123
@enum JointKind UndefinedJointKind FixKind FixTranslationKind RevoluteKind PrismaticKind FreeMotionKind AbsoluteFreeMotionKind

using Unitful

# FixedJoint is used if an Object3D is rigidly fixed to its parent Object3D
struct FixedJoint <: Modia3D.AbstractJoint
end

# FixedTranslationJoint is used if an Object3D is rigidly fixed to its parent Object3D without any rotation
struct FixedTranslationJoint <: Modia3D.AbstractJoint
end


"""
    joint = FreeMotion(; obj1, obj2, path="", r, rot, v, w)

Return a `joint` that describes the free movement of `obj2::`[`Object3D`](@ref)
with respect to `obj1::`[`Object3D`](@ref). The initial position is `r`
(resolved in `obj1`) and the initial orientation is `rot` in [Cardan (Tait–Bryan) angles](https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence)
(rotation sequence x-y-z from `obj1` to `obj2`). `v` (resolved in `obj1`) and `w`
(resolved in `obj2`) are the initial cartesian translational and rotational
velocity vectors.
"""
mutable struct FreeMotion <: Modia3D.AbstractJoint
    path::String

    obj1::Modia3D.AbstractObject3D
    obj2::Modia3D.AbstractObject3D

    ndof::Int

    r::SVector{3,Float64}
    rot::SVector{3,Float64}        # cardan angles
    isrot123::Bool                 # = true: rotation sequence x-y-z, otherwise x-z-y

    v::SVector{3,Float64}
    w::SVector{3,Float64}          # angular velocity vector

    a::SVector{3,Float64}
    z::SVector{3,Float64}          # angular acceleration vector

    residue_f::SVector{3,Float64}
    residue_t::SVector{3,Float64}

    function FreeMotion(; obj1::Modia3D.AbstractObject3D,
                          obj2::Modia3D.AbstractObject3D,
                          path::String = "",
                          r::AbstractVector   = Modia3D.ZeroVector3D(Float64),
                          rot::AbstractVector = Modia3D.ZeroVector3D(Float64),
                          v::AbstractVector   = Modia3D.ZeroVector3D(Float64),
                          w::AbstractVector   = Modia3D.ZeroVector3D(Float64),
                          next_isrot123::Bool = true)  # dummy argument, is ignored

        #(parent,obj,cutJoint) = attach(obj1, obj2)
        #if cutJoint
        #    error("\nError from FreeMotion joint connecting ", Modia3D.fullName(obj1), " with ", Modia3D.fullName(obj2), ":\n",
        #        "    This joint closes a kinematic loop which does not make sense -> remove this joint or correct the kinematic loop!")
        #end
        #
        #if !(obj === obj2)
        #    error("\nError from FreeMotion joint connecting obj1 = ", Modia3D.fullName(obj1), " with obj2 = ", Modia3D.fullName(obj2), ":\n",
        #        "    obj2 has a root, but obj1 has not root. This is currently not supported (exchange obj1 and obj2)!")
        #end

        r   = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"m", r)
        rot = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"rad", rot)
        v   = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"m/s"  , v)
        w   = Modia3D.convertAndStripUnit(SVector{3,Float64}, u"rad/s", w)
        a   = Modia3D.ZeroVector3D(Float64)
        z   = Modia3D.ZeroVector3D(Float64)
        isrot123 = true    # next_isrot123 is ignored.

        residue_f = Modia3D.ZeroVector3D(Float64)
        residue_t = Modia3D.ZeroVector3D(Float64)

        obj2.joint      = new(path, obj1, obj2, 6, r, rot, isrot123, v, w, a, z, residue_f, residue_t)
        obj2.parent     = obj1
        obj2.jointKind  = FreeMotionKind
        obj2.jointIndex = 0
        obj2.ndof       = 6
        obj2.canCollide = true
        obj2.r_rel      = r
        obj2.R_rel      = isrot123 ? Rfromrot123(rot) : Rfromrot132(rot)

        push!(obj1.children, obj2)

        return obj2.joint
    end
end
