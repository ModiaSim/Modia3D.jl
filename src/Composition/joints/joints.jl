# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


# Utility function that should not be directly called (only to be called from attach(..)
function attachAndReverseParents(newParent::Object3D, obj::Object3D)::Nothing
   @assert(!(newParent ≡ obj))

   # Save elements of obj
   r_rel  = obj.r_rel
   R_rel  = obj.R_rel
   parent = obj.parent

   # Reverse obj, so that newParent is the new parent
   obj.parent = newParent
   push!(newParent.children, obj)

   # Reverse all parents of obj
   obj1::Object3D = obj
   obj2::Object3D = parent
   while !(parent === obj1)
      parent_r_rel = parent.r_rel
      parent_R_rel = parent.R_rel

      if r_rel ≡ Modia3D.ZeroVector3D
         parent.r_rel = Modia3D.ZeroVector3D
         parent.r_abs = obj1.r_abs
      else
         parent.r_rel = -R_rel*r_rel
      end

      if R_rel ≡ Modia3D.NullRotation
         parent.R_rel = Modia3D.NullRotation
         parent.R_abs = obj1.R_abs
      else
         parent.R_rel = R_rel'
      end

      obj2 = parent
      parent = obj2.parent
      r_rel  = parent_r_rel
      R_rel  = parent_R_rel
      obj2.parent = obj1
      removeChild!(obj2, obj1)
      push!(obj1.children, obj2)
      obj1 = obj2
   end

   # If !isnothing(scene), visit all objs and set scene
#=
   if !isnothing(scene)
      stack = scene.stack
      empty!(stack)
      append!(stack, obj.children)
      while length(stack) > 0
         obj1 = pop!(stack)
         obj1.scene = scene
         append!(stack, obj1.children)
      end
   end
=#
   return nothing
end


"""
    (obj1, obj2, cutJoint) = attach(frame_a, frame_b)

If frame_a and frame_b do not have the same root, they are attached to each other
and cutJoint = false is returned.

If they have the same root, the tree is not modified and cutJoint=true is returned.
"""
function attach(obj1::Object3D, obj2::Object3D)
   root1 = rootObject3D(obj1)
   root2 = rootObject3D(obj2)
   #println("attach: obj1 = ", Modia3D.fullName(obj1), ", root = ", Modia3D.fullName(root1))
   #println("attach: obj2 = ", Modia3D.fullName(obj2), ", root = ", Modia3D.fullName(root2))

   if root1 ≡ root2
      # Compute absolute positions
      # updatePosition!(root1)
      return (obj1,obj2,true)
   end

   if isWorld(root2)
      attachAndReverseParents(obj2, obj1)
      return (obj2,obj1,false)
   else
      attachAndReverseParents(obj1, obj2)
      return (obj1,obj2,false)
   end
end

#=
function connect(obj1::Object3D, obj2::Object3D;
                 r::AbstractVector = Modia3D.ZeroVector3D,
                 R::Union{Frames.RotationMatrix,Nothing} = nothing,
                 q::Union{Frames.Quaternion,Nothing} = nothing,
                 fixed::Bool = true)::Nothing
   if !isnothing(R) && !isnothing(q)
      error("Modia3D.connect(...): either R or q must be nothing but both have a value.")
   end
   if !isnothing(R)
      Modia3D.assertRotationMatrix(R)
   elseif !isnothing(q)
      Modia3D.assertQuaternion(q)
   end


   (parentObject3D, obj, cutJoint) = attach(obj1, obj2)
  # println("... connect, fixed = ", fixed, ", obj1=",Modia3D.fullName(obj1), ", obj2 = ", Modia3D.fullName(obj2), ", obj = ", Modia3D.fullName(obj))


   if cutJoint
      error("Error from Modia3D.Composition.connect(", obj1.name, ",", obj2.name, "):\n",
            "Not yet supported to rigidly connect two objs that have the same root.")
   end

   r_rel = SVector{3,Float64}(r)
   R_rel = !isnothing(R) ? R                   :
   !isnothing(q) ? Modia3D.from_q(q) : Modia3D.NullRotation

   obj.r_rel = obj===obj2 ? r_rel : -R_rel*r_rel
   obj.R_rel = obj===obj2 ? R_rel :  R_rel'

   #r_abs = parent.r_abs + r_rel
   #R_abs = R_rel*parent.R_abs


   if fixed
      obj.joint = fixedJoint
   else
      obj1.hasChildJoint = true
      q_start = !isnothing(R) ? Modia3D.from_R(R) :
      !isnothing(q) ? q                   : Modia3D.NullQuaternion
      q_start = obj===obj2 ? q_start : Modia3D.inverseRotation(q_start)

      obj.joint = FreeMotion(obj, r_start = obj.r_rel, q_start = q_start)
   end
   return nothing
end
=#



function updatePosition!(obj::Object3D)::Nothing
   stack = Object3D[]
   # Push initial children on stack
   append!(stack, obj.children)
   while length(stack) > 0
      obj = pop!(stack)

      # computeKinematics!(obj.joint, obj, Modia3D.KinematicAnalysis, 0.0)

      parent = obj.parent

      if obj.r_rel ≡ Modia3D.ZeroVector3D
         obj.r_abs = parent.r_abs
      else
         obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
      end

      if obj.R_rel ≡ Modia3D.NullRotation
         obj.R_abs = parent.R_abs
      else
         obj.R_abs = obj.R_rel*parent.R_abs
      end

      if length(obj.visualizationFrame) == 1
         obj.visualizationFrame[1].r_abs = obj.r_abs
         obj.visualizationFrame[1].R_abs = obj.R_abs
      end

      append!(stack, obj.children)
   end
   return nothing
end




"""
    R = Rfromrot123(rot123::AbstractVector)

Return rotation matrix `R` from Cardan angles (rotation sequence x-y-z).
`rot123` are the Cardan angles (rotation sequence x-y-z) of rotation from frame `1` to frame `2`.
"""
function Rfromrot123(rot123::AbstractVector)

    (sal, cal) = sincos(rot123[1])
    (sbe, cbe) = sincos(rot123[2])
    (sga, cga) = sincos(rot123[3])
    return @SMatrix[ cbe*cga   sal*sbe*cga+cal*sga  -cal*sbe*cga+sal*sga ;
                    -cbe*sga  -sal*sbe*sga+cal*cga   cal*sbe*sga+sal*cga ;
                     sbe      -sal*cbe               cal*cbe             ]

end


"""
    R = Rfromrot132(rot132::AbstractVector)

Return rotation matrix `R` from Cardan angles (rotation sequence x-z-y).
`rot132` are the Cardan angles (rotation sequence x-z-y) of rotation from frame `1` to frame `2`.
"""
function Rfromrot132(rot132::AbstractVector)

    (sal, cal) = sincos(rot132[1])
    (sga, cga) = sincos(rot132[2])
    (sbe, cbe) = sincos(rot132[3])
    return @SMatrix[ cbe*cga   cal*cbe*sga+sal*sbe   sal*cbe*sga-cal*sbe ;
                        -sga   cal*cga               sal*cga             ;
                     sbe*cga   cal*sbe*sga-sal*cbe   sal*sbe*sga+cal*cbe ]

end



"""
    w = wfromrot123(rot123::AbstractVector, derrot123::AbstractVector)

Return relative rotational velocity Vector3D `w` from frame `1` to frame `2` resolved in frame `2`.

`rot123` are the Cardan angles (rotation sequence x-y-z) of rotation from frame `1` to frame `2`.
`derrot123` are the time derivatives of `rot123`.
"""
function wfromrot123(rot123::AbstractVector, derrot123::AbstractVector)

    # 2_w_12 = (0, 0, gad) + Rga21*[(0, bed, 0) + Rbe21*(ald, 0, 0)]
    #        = (cbe*cga*ald + sga*bed, -cbe*sga*ald + cga*bed, sbe*ald + gad)
    (sbe, cbe) = sincos(rot123[2])
    (sga, cga) = sincos(rot123[3])
    return @SVector[ cbe*cga*derrot123[1] + sga*derrot123[2]                ,
                    -cbe*sga*derrot123[1] + cga*derrot123[2]                ,
                         sbe*derrot123[1]                    + derrot123[3] ]

end

# Next function only for backwards compatibility (do not use for new model)
computeKinematics!(scene::Scene, joint::Modia3D.AbstractJoint, obj::Object3D, analysis::Modia3D.AnalysisType, time::Float64)::Nothing =
    computeKinematics!(scene, [obj], time)


"""
    computeKinematics!(scene::Scene, tree::Vector{Object3D}, time)

Compute position, velocity, acceleration variables of the Object3Ds that are connected
in form of a tree. Variable `tree` contains the Object3Ds in a traversal order (e.g. pre-order traversal).
`tree[1]` is the root object. It is assumed that the kinematic
variables of tree[1].parent have a meaningful value.
"""
function computeKinematics!(scene::Scene, tree::Vector{Object3D}, time)::Nothing
    for obj in tree
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == FixTranslationKind
            obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
            obj.R_abs = parent.R_abs

            obj.v0 = parent.v0 + parent.R_abs'*cross(parent.w, obj.r_rel)
            obj.a0 = parent.a0 + parent.R_abs'*(cross(parent.z, obj.r_rel) + cross(parent.w, cross(parent.w, obj.r_rel)))

            obj.w = parent.w
            obj.z = parent.z

        elseif jointKind == FixKind
            obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
            obj.R_abs = obj.R_rel*parent.R_abs

            obj.v0 = parent.v0 + parent.R_abs'*cross(parent.w, obj.r_rel)
            obj.a0 = parent.a0 + parent.R_abs'*(cross(parent.z, obj.r_rel) + cross(parent.w, cross(parent.w, obj.r_rel)))

            obj.w = obj.R_rel*parent.w
            obj.z = obj.R_rel*parent.z

        elseif jointKind == RevoluteKind
            revolute  = scene.revolute[obj.jointIndex]

            obj.r_abs = parent.r_abs
            obj.R_rel = Frames.rotAxis(revolute.posAxis, revolute.posMovement, revolute.phi)
            obj.R_abs = obj.R_rel*parent.R_abs

            obj.v0 = parent.v0
            obj.a0 = parent.a0

            w_rel = Frames.axisValue(revolute.posAxis, revolute.posMovement, revolute.w)
            z_rel = Frames.axisValue(revolute.posAxis, revolute.posMovement, revolute.a)

            obj.w = obj.R_rel*(parent.w + w_rel)
            obj.z = obj.R_rel*(parent.z + z_rel + cross(parent.w, w_rel))

        elseif jointKind == PrismaticKind
            prismatic = scene.prismatic[obj.jointIndex]

            obj.r_rel = prismatic.eAxis*prismatic.s
            obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
            obj.R_abs = parent.R_abs

            v_rel  = prismatic.eAxis*prismatic.v
            a_rel  = prismatic.eAxis*prismatic.a
            obj.v0 = parent.v0 + parent.R_abs'*(v_rel + cross(parent.w, obj.r_rel))
            obj.a0 = parent.a0 + parent.R_abs'*(a_rel + cross(parent.z, obj.r_rel) + cross(parent.w, v_rel + cross(parent.w, obj.r_rel)) )

            obj.w  = parent.w
            obj.z  = parent.z

        elseif jointKind == AbsoluteFreeMotionKind
            freeMotion  = scene.freeMotion[obj.jointIndex]

            obj.r_rel = freeMotion.r
            obj.R_rel = Rfromrot123(freeMotion.rot)

            obj.r_abs = obj.r_rel
            obj.R_abs = obj.R_rel

            obj.v0 = freeMotion.v
            obj.a0 = freeMotion.a

            obj.w  = freeMotion.w
            obj.z  = freeMotion.z

        elseif jointKind == FreeMotionKind
            freeMotion  = scene.freeMotion[obj.jointIndex]

            obj.r_rel = freeMotion.r
            obj.R_rel = Rfromrot123(freeMotion.rot)

            obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
            obj.R_abs = obj.R_rel*parent.R_abs

            obj.v0 = parent.v0 + parent.R_abs'*(freeMotion.v + cross(parent.w, obj.r_rel))
            obj.a0 = parent.a0 + parent.R_abs'*(freeMotion.a + cross(parent.z, obj.r_rel) +
                                                    cross(parent.w, cross(parent.w, obj.r_rel)))
            obj.w  = parent.R_abs*parent.w + freeMotion.w
            obj.z  = parent.R_abs*parent.z + freeMotion.z

        else
            error("Bug in Modia3D/src/Composition/specifics/specifics.jl (computeKinematics!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



"""
    computeKinematics_for_leq_mode_pos!(scene::Scene, tree::Vector{Object3D}, time)

Compute accelerations that are only a function of qdd, but not of q and qd.
of the Object3Ds that are connected in form of a tree.
"""
function computeKinematics_for_leq_mode_pos!(scene::Scene, tree::Vector{Object3D}, time)::Nothing
    for obj in tree
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == FixTranslationKind
            obj.a0 = parent.a0 + parent.R_abs'*(cross(parent.z, obj.r_rel))
            obj.z  = parent.z

        elseif jointKind == FixKind
            obj.a0 = parent.a0 + parent.R_abs'*(cross(parent.z, obj.r_rel))
            obj.z  = obj.R_rel*parent.z

        elseif jointKind == RevoluteKind
            revolute = scene.revolute[obj.jointIndex]

            obj.a0 = parent.a0
            z_rel  = Frames.axisValue(revolute.posAxis, revolute.posMovement, revolute.a)
            obj.z  = obj.R_rel*(parent.z + z_rel)

        elseif jointKind == PrismaticKind
            prismatic = scene.prismatic[obj.jointIndex]

            eAxis  = prismatic.posMovement ? prismatic.posAxis : -prismatic.posAxis
            a_rel  = prismatic.eAxis*prismatic.a
            obj.a0 = parent.a0 + parent.R_abs'*(a_rel + cross(parent.z, obj.r_rel))
            obj.z  = parent.z

        elseif jointKind == AbsoluteFreeMotionKind
            freeMotion = scene.freeMotion[obj.jointIndex]

            obj.a0 = freeMotion.a
            obj.z  = freeMotion.z

        elseif jointKind == FreeMotionKind
            freeMotion = scene.freeMotion[obj.jointIndex]

            obj.a0 = parent.a0 + parent.R_abs'*(freeMotion.a + cross(parent.z, obj.r_rel))
            obj.z  = parent.R_abs*parent.z + freeMotion.z

        else
            error("Bug in Modia3D/src/Composition/specifics/specifics.jl (computeKinematics_for_leq_mode_pos!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



"""
    computeForcesTorquesAndResiduals!(scene::Scene, tree::Vector{Object3D}, time)

Compute forces/torques and residuals in a backward recursion from tree[end] to tree[1].
Variable `tree` contains the Object3Ds in a traversal order (e.g. pre-order traversal).
It is assumed that all force/torque variables are initialized (e.g. to zero), including
tree[1].parent.
"""
function computeForcesTorquesAndResiduals!(scene::Scene, tree::Vector{Object3D}, time)::Nothing
    for i = length(tree):-1:1
        obj       = tree[i]
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == FixTranslationKind
            parent.f += obj.f
            parent.t += obj.t + cross(obj.r_rel, obj.f)

        elseif jointKind == FixKind
            parent.f += obj.R_rel'*obj.f
            parent.t += obj.R_rel'*obj.t + cross(obj.r_rel, obj.f)

        elseif jointKind == RevoluteKind
            revolute  = scene.revolute[obj.jointIndex]
            parent.f += obj.R_rel'*obj.f
            parent.t += obj.R_rel'*obj.t
            obj.f     = -obj.f
            obj.t     = -obj.t
            revolute.residue = -revolute.tau + (revolute.posMovement ? obj.t[revolute.posAxis] : -obj.t[revolute.posAxis])

        elseif jointKind == PrismaticKind
            prismatic = scene.prismatic[obj.jointIndex]
            parent.f += obj.f
            parent.t += obj.t + cross(obj.r_rel, obj.f)
            obj.f     = -obj.f
            obj.t     = -obj.t
            prismatic.residue = -prismatic.f + dot(prismatic.eAxis,obj.f)

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            freeMotion           = scene.freeMotion[obj.jointIndex]
            freeMotion.residue_f = obj.f
            freeMotion.residue_t = obj.t

        else
            error("Bug in Modia3D/src/Composition/specifics/specifics.jl (computeForcesTorquesAndResiduals!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



"""
    setJointVariables_q_qd_f!(scene::Scene, objects::Vector{Object3D},
                              startIndex::Vector{Int}, ndof::Vector{Int}, args)

Copy generalized joints variables (q,qd,f) into the corresponding Object3Ds.
"""
function setJointVariables_q_qd_f!(scene::Scene, objects::Vector{Object3D}, startIndex::Vector{Int},
                                   ndof::Vector{Int}, args)::Nothing
    for (i,obj) in enumerate(objects)
        jointKind = obj.jointKind
        args_i    = args[i]

        if jointKind == RevoluteKind
            @assert(ndof[i] == 1)
            revolute     = scene.revolute[obj.jointIndex]
            revolute.phi = args_i[1]
            revolute.w   = args_i[2]
            revolute.tau = args_i[3]

        elseif jointKind == PrismaticKind
            @assert(ndof[i] == 1)
            prismatic   = scene.prismatic[obj.jointIndex]
            prismatic.s = args_i[1]
            prismatic.v = args_i[2]
            prismatic.f = args_i[3]

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            @assert(ndof[i] == 6)
            freeMotion     = scene.freeMotion[obj.jointIndex]
            freeMotion.r   = SVector{3,Float64}(args_i[1])
            freeMotion.rot = SVector{3,Float64}(args_i[2])
            freeMotion.v   = SVector{3,Float64}(args_i[3])
            freeMotion.w   = SVector{3,Float64}(args_i[4])
            
        else
           error("Bug in Modia3D/src/Composition/specifics/specifics.jl (setJointVariables!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



"""
    setJointVariables_qdd!(scene::Scene, objects::Vector{Object3D}, startIndex::Vector{Int},
                           ndof::Vector{Int}, qdd)

Copy generalized joint accelerations into the corresponding joints.
"""
function setJointVariables_qdd!(scene::Scene, objects::Vector{Object3D}, startIndex::Vector{Int},
                                ndof::Vector{Int}, qdd)::Nothing

    for (i,obj) in enumerate(objects)
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            @assert(ndof[i] == 1)
            revolute   = scene.revolute[obj.jointIndex]
            revolute.a = qdd[startIndex[i]]

        elseif jointKind == PrismaticKind
            @assert(ndof[i] == 1)
            prismatic   = scene.prismatic[obj.jointIndex]
            prismatic.a = qdd[startIndex[i]]

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            @assert(ndof[i] == 6)
            qdd2::Vector{Float64} = qdd
            beg          = startIndex[i]
            freeMotion   = scene.freeMotion[obj.jointIndex]
            freeMotion.a = SVector{3,Float64}(qdd2[beg]  , qdd2[beg+1], qdd2[beg+2])
            freeMotion.z = SVector{3,Float64}(qdd2[beg+3], qdd2[beg+4], qdd2[beg+5])

        else
           error("Bug in Modia3D/src/Composition/specifics/specifics.jl (setJointVariables!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



"""
    getJointResiduals_for_leq_mode_0!(scene::Scene, objects::Vector{Object3D}, residuals, startIndex::Int, ndof::Int, cache_h)

Copy specific variables into their objects for leq_mode = 0.
"""
function getJointResiduals_for_leq_mode_0!(scene::Scene, objects::Vector{Object3D}, residuals, startIndex::Vector{Int}, ndof::Vector{Int}, cache_h)::Nothing
    for (i,obj) in enumerate(objects)
        jointKind = obj.jointKind
        beg       = startIndex[i]

        if jointKind == RevoluteKind
            @assert(ndof[i] == 1)
            revolute       = scene.revolute[obj.jointIndex]
            cache_h[  beg] = revolute.residue + revolute.tau
            residuals[beg] = revolute.residue

        elseif jointKind == PrismaticKind
            @assert(ndof[i] == 1)
            prismatic      = scene.prismatic[obj.jointIndex]
            cache_h[  beg] = prismatic.residue + prismatic.f
            residuals[beg] = prismatic.residue

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            @assert(ndof[i] == 6)
            freeMotion             = scene.freeMotion[obj.jointIndex]
            cache_h[  beg+0:beg+2] = freeMotion.residue_f
            cache_h[  beg+3:beg+5] = freeMotion.residue_t

            residuals[beg+0:beg+2] = freeMotion.residue_f
            residuals[beg+3:beg+5] = freeMotion.residue_t

        else
           error("Bug in Modia3D/src/Composition/specifics/specifics.jl (getJointResiduals_for_leq_mode_0!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



"""
    getJointResiduals_for_leq_mode_pos!(scene::Scene, objects::Vector{Object3D}, residuals, startIndex::Int, ndof::Int, cache_h)

Copy specific variables into their objects for leq_mode > 0.
"""
function getJointResiduals_for_leq_mode_pos!(scene::Scene, objects::Vector{Object3D}, residuals, startIndex::Vector{Int}, ndof::Vector{Int}, cache_h)::Nothing
    for (i,obj) in enumerate(objects)
        jointKind = obj.jointKind
        beg       = startIndex[i]

        if jointKind == RevoluteKind
            @assert(ndof[i] == 1)
            residuals[beg] = scene.revolute[obj.jointIndex].residue + cache_h[beg]

        elseif jointKind == PrismaticKind
            @assert(ndof[i] == 1)
            residuals[beg] = scene.prismatic[obj.jointIndex].residue + cache_h[beg]

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            @assert(ndof[i] == 6)
            freeMotion             = scene.freeMotion[obj.jointIndex]
            residuals[beg+0:beg+2] = freeMotion.residue_f + cache_h[beg+0:beg+2]
            residuals[beg+3:beg+5] = freeMotion.residue_t + cache_h[beg+3:beg+5]

        else
           error("Bug in Modia3D/src/Composition/specifics/specifics.jl (getJointResiduals_for_leq_mode_0!): jointKind = $jointKind is not known")
        end
    end
    return nothing
end



# For backwards compatibility (do not use for new models)
function setAngle!(revolute::Revolute, phi::Float64)
   obj          = revolute.obj2
   revolute.phi = phi
   obj.R_rel    = Frames.rotAxis(revolute.posAxis, revolute.posMovement, phi)
   obj.R_abs    = obj.R_rel*obj.parent.R_abs
   return revolute
end


# For backwards compatibility (do not use for new models)
function setDistance!(prismatic::Prismatic, s::Float64)
   obj         = prismatic.obj2
   prismatic.s = s
   obj.r_rel   = prismatic.eAxis*s
   obj.r_abs   = parent.r_abs + parent.R_abs'*obj.r_rel
   return prismatic
end
