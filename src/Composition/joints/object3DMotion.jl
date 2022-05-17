@enum RotationVariables               RCardan123
@enum WStartVariables    WCartesian   WCardan123
@enum JointKind UndefinedJointKind FixKind FixTranslationKind RevoluteKind PrismaticKind FreeMotionKind AbsoluteFreeMotionKind

using Unitful

# FixedJoint is used if an Object3D is rigidly fixed to its parent Object3D
struct FixedJoint{F <: Modia3D.VarFloatType} <: Modia3D.AbstractJoint
end

# FixedTranslationJoint is used if an Object3D is rigidly fixed to its parent Object3D without any rotation
struct FixedTranslationJoint{F <: Modia3D.VarFloatType} <: Modia3D.AbstractJoint
end


"""
    J = J123(rot123::AbstractVector)

Return joint rot. kinematics matrix `J` for Cardan angles `rot123` (rotation sequence x-y-z).
"""
function J123(rot123::SVector{3,F})::SMatrix{3,3,F,9} where {F}
    (sbe, cbe) = sincos(rot123[2])
    (sga, cga) = sincos(rot123[3])
    return SMatrix{3,3,F,9}(cga/cbe, sga, -sbe*cga/cbe, -sga/cbe, cga, sbe*sga/cbe, F(0.0), F(0.0), F(1.0))
end

"""
    J = J132(rot132::AbstractVector)

Return joint rot. kinematics matrix `J` for Cardan angles `rot132` (rotation sequence x-z-y).
"""
function J132(rot132::SVector{3,F})::SMatrix{3,3,F,9} where {F}
    (sga, cga) = sincos(rot132[2])
    (sbe, cbe) = sincos(rot132[3])
    return SMatrix{3,3,F,9}(cbe/cga, -sbe, cbe*sga/cga, F(0.0), F(0.0), F(1.0), sbe/cga, cbe, sbe*sga/cga)
end

J123or132(rot, isrot123) = isrot123 ? J123(rot) : J132(rot)



"""
    joint = FreeMotion(; obj1, obj2, r, rot, v, w)

Return a `joint` that describes the free movement of `obj2::`[`Object3D`](@ref)
with respect to `obj1::`[`Object3D`](@ref). The initial position is `r`
(resolved in `obj1`) and the initial orientation is `rot` in [Cardan (Tait–Bryan) angles](https://en.wikipedia.org/wiki/Euler_angles#Chained_rotations_equivalence)
(rotation sequence x-y-z from `obj1` to `obj2`). `v` (resolved in `obj1`) and `w`
(resolved in `obj2`) are the initial cartesian translational and rotational
velocity vectors.
"""
mutable struct FreeMotion{F <: Modia3D.VarFloatType} <: Modia3D.AbstractJoint
    path::String
    
    # Hidden states are stored in the following way:
    #   x_hidden[ix_r:...] = [r, rot, v, w]
    #
    # Derivatives of hidden states are stored in the following way
    #   der_x_hidden[ix_r:...] = [der(r), der(rot), der(v), der(w)]
    #
    # with
    #   der(r)   = v
    #   der(rot) = J123or132(rot,isrot123) * w
    #   der(v)   = ...
    #   der(w)   = ...
    
    hiddenStates::Bool   # = true, if state is not visible in generated code
    ix_hidden_r::Int    # instantiatedModel.x_hidden[ix_r_hidden  :ix_hidden_r+2]     are the elements of x_hidden that are stored in r   if hiddenStates
    ix_hidden_rot::Int  # instantiatedModel.x_hidden[ix_rot_hidden:ix_hidden_rot+2] are the elements of x_hidden that are stored in rot if hiddenStates
    ix_hidden_v::Int    # instantiatedModel.x_hidden[ix_v_hidden  :ix_hidden_v+2]     are the elements of x_hidden that are stored in v   if hiddenStates
    ix_hidden_w::Int    # instantiatedModel.x_hidden[ix_w_hidden  :ix_hidden_w+2]     are the elements of x_hidden that are stored in w   if hiddenStates
    iz_rot2::Int        # instantiatedModel.eventHandler.z[iz_rot2] is the element of z in which singularRem(rot[2]) is stored if hiddenStates, 
                        # to monitor when to switch to a different rotation sequence of rot
    iqdd_hidden::Int    # qdd_hidden[iqdd_hidden:iqdd_hidden+5] are the elements of qdd that are stored in [a,z] if hiddenStates
    
    obj1::Modia3D.AbstractObject3D
    obj2::Modia3D.AbstractObject3D

    ndof::Int

    r::SVector{3,F}
    rot::SVector{3,F}        # cardan angles
    isrot123::Bool           # = true: rotation sequence x-y-z, otherwise x-z-y

    v::SVector{3,F}          # = der(r)
    w::SVector{3,F}          # angular velocity vector

    a::SVector{3,F}          # = der(v)
    z::SVector{3,F}          # = der(w)

    function FreeMotion{F}(; obj1::Modia3D.AbstractObject3D,
                             obj2::Modia3D.AbstractObject3D,
                             path::String = "",
                             r::AbstractVector   = Modia3D.ZeroVector3D(F),
                             rot::AbstractVector = Modia3D.ZeroVector3D(F),
                             v::AbstractVector   = Modia3D.ZeroVector3D(F),
                             w::AbstractVector   = Modia3D.ZeroVector3D(F),
                             next_isrot123::Bool = true,
                             hiddenStates::Bool = false
                          ) where F <: Modia3D.VarFloatType

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

        r   = Modia3D.convertAndStripUnit(SVector{3,F}, u"m", r)
        rot = Modia3D.convertAndStripUnit(SVector{3,F}, u"rad", rot)
        v   = Modia3D.convertAndStripUnit(SVector{3,F}, u"m/s"  , v)
        w   = Modia3D.convertAndStripUnit(SVector{3,F}, u"rad/s", w)
        a   = Modia3D.ZeroVector3D(F)
        z   = Modia3D.ZeroVector3D(F)
        isrot123 = true

        obj2.joint      = new(path, hiddenStates, -1, -1, -1, -1, -1, -1, obj1, obj2, 6, r, rot, isrot123, v, w, a, z)
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
