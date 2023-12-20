# License for this file: MIT (expat)
# Copyright 2017-2021, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

mutable struct MultibodyData{F <: Modia3D.VarFloatType, TimeType}
    instantiatedModel::Modia.InstantiatedModel{F,TimeType}
    path::String                              # Path of Model3D(...)

    world::Object3D{F}                        # Pointer to world object
    scene::Scene{F}                           # Pointer to scene

    revolute::Vector{Revolute{F}}             # References to revolute joints
    prismatic::Vector{Prismatic{F}}           # References to prismatic joints
    freeMotion::Vector{FreeMotion{F}}         # References to freeMotion joints (joint.hidden_qdd_startIndex >= 0)

    revoluteObjects::Vector{Object3D{F}}      # References to Object3Ds with a Revolute joint
    prismaticObjects::Vector{Object3D{F}}     # References to Object3Ds with a Prismatic joint
    freeMotionObjects::Vector{Object3D{F}}    # References to Object3Ds with a FreeMotion joint and joint.hiddenStates = false
    hiddenJointObjects::Vector{Object3D{F}}   # References to Object3Ds with a joint that has joint.hiddenStates = true

    revoluteGenForces::Vector{F}              # = M_r*der(qd_r) + h_r(q,qd,gravity,contact-forces) = <generalized driving torques>
    prismaticGenForces::Vector{F}             # = M_p*der(qd_p) + h_p(q,qd,gravity,contact-forces) = <generalized driving forces>
    genForces::Vector{F}                      # = [revoluteGenForces, prismaticGenForces]
    freeMotionGenForces::Vector{SVector{3,F}} # = M_f*der(qd_f) + h_f(q,qd,gravity,contact-forces) = 0             (joint.hiddenStates = false)
    hiddenGenForces::Vector{F}                # = M_hidden*qdd_hidden + h_hidden(q,qd,gravity,contact-forces) = 0  (joint.hiddenStates = true)

    genForcesCache_h::Vector{F}               # = [h_r, h_p]
    freeMotionCache_h::Vector{SVector{3,F}}   # = h_f(q,qd,gravity,contact-forces)
    hiddenCache_h::Vector{F}                  # = h_hidden(q,qd,gravity,contact-forces)

    objIndices::Matrix{Int}                   # objIndices[i,1]: Index of local w_segmented variable r_abs of Object3D i
                                              #           [i,2]: Index of local w_segmented variable R_abs of Object3D i
    zStartIndex::Int                          # eventHandler.z[zStartIndex] is first index of crossing function
                                              # (or zero, if enableContactDetection=false)
    nz::Int                                   # Number of used zero crossing functions
    time::TimeType                            # Current time

    freeMotionResiduals::Vector{F}            # freeMotionGenForces with elements of type F instead of type SVector{3,F}

    # for multibodyAccelerations
    leq::Vector{Modia.LinearEquations{F}}

    function MultibodyData{F,TimeType}(partiallyInstantiatedModel::Modia.InstantiatedModel{F,TimeType}, modelPath::String, world, scene,
                                       revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects,
                                       revoluteIndices, prismaticIndices, freeMotionIndices, objIndices,
                                       zStartIndex, nz) where {F,TimeType}
        # Make joints available
        revoluteJoints   = Vector{Revolute{F}}(  undef, length(revoluteObjects))
        prismaticJoints  = Vector{Prismatic{F}}( undef, length(prismaticObjects))
        freeMotionJoints = Vector{FreeMotion{F}}(undef, length(freeMotionObjects) + length(hiddenJointObjects))

        for obj in revoluteObjects
            obj.jointIndex = revoluteIndices[obj.joint.path]
            revoluteJoints[obj.jointIndex] = obj.joint
        end

        for obj in prismaticObjects
            obj.jointIndex = prismaticIndices[obj.joint.path]
            prismaticJoints[obj.jointIndex] = obj.joint
        end

        for obj in freeMotionObjects
            obj.jointIndex = freeMotionIndices[obj.joint.path]
            freeMotionJoints[obj.jointIndex] = obj.joint

            if hasNoParent(obj.parent)
                obj.jointKind = AbsoluteFreeMotionKind
            end
        end

        # Code below assumes, that all hiddenJointObjects are FreeMotion joints
        i = 1
        j = length(freeMotionObjects) + 1
        path::String = ""
        for obj in hiddenJointObjects
            joint = obj.joint
            @assert(typeof(joint) <: FreeMotion)
            freeMotionJoints[j] = obj.joint
            freeMotion = freeMotionJoints[j]
            freeMotion.iqdd_hidden = i
            obj.jointIndex = j
            i += 6
            j += 1

            if hasNoParent(obj.parent)
                obj.jointKind = AbsoluteFreeMotionKind
            end

            # Define hidden model states and copy initial values into eqInfo
            path = obj.path * "."
            w_init = freeMotion.wResolvedInParent ? Modia3D.resolve1(freeMotion.rot, freeMotion.w, rotationXYZ=freeMotion.isrot123) : freeMotion.w
            freeMotion.ix_segmented_r     = Modia.new_x_segmented_variable!(partiallyInstantiatedModel, path*"translation"    , path*"der(translation)"    , freeMotion.r)
            freeMotion.ix_segmented_v     = Modia.new_x_segmented_variable!(partiallyInstantiatedModel, path*"velocity"       , path*"der(velocity)"       , freeMotion.v)
            freeMotion.ix_segmented_rot   = Modia.new_x_segmented_variable!(partiallyInstantiatedModel, path*"rotation"       , path*"der(rotation)"       , freeMotion.rot)
            freeMotion.ix_segmented_w     = Modia.new_x_segmented_variable!(partiallyInstantiatedModel, path*"angularVelocity", path*"der(angularVelocity)", w_init)
            freeMotion.iextra_isrot123 = Modia.new_w_segmented_variable!(partiallyInstantiatedModel, path*"rotationXYZ", freeMotion.isrot123)
            freeMotion.ix_rot          = Modia.get_x_startIndex_from_x_segmented_startIndex(partiallyInstantiatedModel,freeMotion.ix_segmented_rot)

            # Define event indicator to monitor changing sequence of rotation angles
            freeMotion.iz_rot2 = Modia.new_z_segmented_variable!(partiallyInstantiatedModel, 1)
        end

        for obj in scene.treeForComputation
            if obj.jointKind == FixKind
                if obj.R_rel === Modia3D.NullRotation(F)
                    obj.jointKind = FixTranslationKind
                end
            end
        end
        nhidden_qdd = 6*length(hiddenJointObjects)

        new(partiallyInstantiatedModel, modelPath, world, scene,
            revoluteJoints , prismaticJoints , freeMotionJoints,
            revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects,
            zeros(F, length(revoluteObjects)), zeros(F, length(prismaticObjects)),
            zeros(F, length(revoluteObjects)+length(prismaticObjects)), zeros(SVector{3,F}, 2*length(freeMotionObjects)), zeros(F, nhidden_qdd),
            zeros(F, length(revoluteObjects)+length(prismaticObjects)), zeros(SVector{3,F}, 2*length(freeMotionObjects)), zeros(F, nhidden_qdd),
            objIndices, zStartIndex, nz, partiallyInstantiatedModel.time,
            zeros(F, 2*3*length(freeMotionObjects)),
            Modia.LinearEquations{F}[])
    end
end

mutable struct MultibodyBuild{F <: Modia3D.VarFloatType, TimeType}
    Model3DPath::String                         # Path of the Model3D(..) command used to define the system, e.g. "a.b.c"
    revoluteIndices::OrderedDict{String,Int}    # obj.jointIndex = revoluteIndices["a.b.c"] (= order of arguments in setStatesRevolute!(..., args...))
    prismaticIndices::OrderedDict{String,Int}   # obj.jointIndex = prismaticIndices["a.b.c"] (= order of arguments in setStatesPrismatic!(..., args...))
    freeMotionIndices::OrderedDict{String,Int}  # obj.jointIndex = freeMotionIndices["a.b.c"] (= order of arguments in setStatesFreeMotion!(..., args...))
    mbs::Union{MultibodyData{F,TimeType}, Nothing}

    MultibodyBuild{F,TimeType}(Model3DPath::String, revoluteIndices, prismaticIndices, freeMotionIndices) where {F,TimeType} =
                              new(Model3DPath, revoluteIndices, prismaticIndices, freeMotionIndices, nothing)
end


# Utility function that should not be directly called (only to be called from attach(..)
function attachAndReverseParents(newParent::Object3D{F}, obj::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
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

      if r_rel ≡ Modia3D.ZeroVector3D(F)
         parent.r_rel = Modia3D.ZeroVector3D(F)
         parent.r_abs = obj1.r_abs
      else
         parent.r_rel = -R_rel*r_rel
      end

      if R_rel ≡ Modia3D.NullRotation(F)
         parent.R_rel = Modia3D.NullRotation(F)
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


getParents(obj,rootPath) = "\"" * Modia3D.fullName(obj) * "\" has " * (length(rootPath) == 0 ? "no parents" : "parents: $rootPath")

"""
    (obj1, obj2, cutJoint) = attach(frame_a, frame_b)

If frame_a and frame_b do not have the same root, they are attached to each other
and cutJoint = false is returned.

If they have the same root, the tree is not modified and cutJoint=true is returned.
"""
function attach(obj1::Object3D, obj2::Object3D; name = "")
   root1 = rootObject3D(obj1)
   root2 = rootObject3D(obj2)
   #println("attach: obj1 = ", Modia3D.fullName(obj1), ", root = ", Modia3D.fullName(root1))
   #println("attach: obj2 = ", Modia3D.fullName(obj2), ", root = ", Modia3D.fullName(root2))

   if root1 ≡ root2
      # Compute absolute positions
      # updatePosition!(root1)

      if name != ""
        # Collect all objects that form a loop
        rootPath1 = rootObject3DPath(obj1)
        rootPath2 = rootObject3DPath(obj2)
        error("\nError from $name connecting \"", Modia3D.fullName(obj1), "\" with \"", Modia3D.fullName(obj2), "\":\n",
                "    ", getParents(obj1,rootPath1), "\n",
                "    ", getParents(obj2,rootPath2), "\n",
                "    Therefore, a closed kinematic loop is defined, which is currently not supported.")
      end
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


function updatePosition!(obj::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
   stack = Object3D[]
   # Push initial children on stack
   append!(stack, obj.children)
   while length(stack) > 0
      obj = pop!(stack)

      # computeKinematics!(obj.joint, obj, Modia3D.KinematicAnalysis, 0.0)

      parent = obj.parent

      if obj.r_rel ≡ Modia3D.ZeroVector3D(F)
         obj.r_abs = parent.r_abs
      else
         obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
      end

      if obj.R_rel ≡ Modia3D.NullRotation(F)
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

Return relative rotational velocity SVector{3,F} `w` from frame `1` to frame `2` resolved in frame `2`.

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


"""
    computePositionsVelocitiesAccelerations!(mbs::MultibodyData{F}, tree::Vector{Object3D{F}}, time) where F <: Modia3D.VarFloatType

Compute position, velocity, acceleration variables of the Object3Ds that are connected
in form of a tree. Variable `tree` contains the Object3Ds in a traversal order (e.g. pre-order traversal).
`tree[1]` is the root object. It is assumed that the kinematic
variables of tree[1].parent have a meaningful value.
"""
function computePositionsVelocitiesAccelerations!(mbs::MultibodyData{F,TimeType}, tree::Vector{Object3D{F}}, time)::Nothing where {F <: Modia3D.VarFloatType, TimeType}
    @inbounds for obj in tree
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            revolute  = mbs.revolute[obj.jointIndex]

            obj.r_abs = parent.r_abs
            obj.R_rel = Frames.rot_e(revolute.eAxis, revolute.phi)
            obj.R_abs = obj.R_rel*parent.R_abs

            obj.v0 = parent.v0
            obj.a0 = parent.a0

            w_rel = revolute.eAxis*revolute.w
            z_rel = revolute.eAxis*revolute.a

            obj.w = obj.R_rel*(parent.w + w_rel)
            obj.z = obj.R_rel*(parent.z + z_rel + cross(parent.w, w_rel))

        elseif jointKind == PrismaticKind
            prismatic = mbs.prismatic[obj.jointIndex]

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
            freeMotion  = mbs.freeMotion[obj.jointIndex]

            obj.r_rel = freeMotion.r
            obj.R_rel = freeMotion.isrot123 ? Rfromrot123(freeMotion.rot) : Rfromrot132(freeMotion.rot)

            obj.r_abs = obj.r_rel
            obj.R_abs = obj.R_rel

            obj.v0 = freeMotion.v
            obj.a0 = freeMotion.a

            obj.w  = freeMotion.w
            obj.z  = freeMotion.z

        elseif jointKind == FreeMotionKind
            freeMotion  = mbs.freeMotion[obj.jointIndex]

            obj.r_rel = freeMotion.r
            obj.R_rel = freeMotion.isrot123 ? Rfromrot123(freeMotion.rot) : Rfromrot132(freeMotion.rot)

            obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
            obj.R_abs = obj.R_rel*parent.R_abs

            obj.v0 = parent.v0 + parent.R_abs'*(freeMotion.v + cross(parent.w, obj.r_rel))
            obj.a0 = parent.a0 + parent.R_abs'*(freeMotion.a + cross(parent.z, obj.r_rel) +
                                                    cross(parent.w, cross(parent.w, obj.r_rel)))
            obj.w  = parent.R_abs*parent.w + freeMotion.w
            obj.z  = parent.R_abs*parent.z + freeMotion.z

        elseif jointKind == FixTranslationKind
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

        else
            error("Bug in Modia3D/src/Composition/joints/joints.jl (computePositionsVelocitiesAccelerations!): jointKind = $jointKind is not known.")
        end
    end
    return nothing
end



"""
    computeAccelerations!(mbs::MultibodyData{F,TimeType}, tree::Vector{Object3D{F}}, time)

Traverse the tree of Object3Ds from world to leaf objects and compute the generalized accelerations of the
joints and the accelerations of the Object3Ds. Hereby it is assumed that the generalized position/velocities
of the joints and the position/velocities of the Object3Ds are already stored in the Object3Ds/joints.
"""
function computeAccelerations!(mbs::MultibodyData{F,TimeType}, tree::Vector{Object3D{F}}, time)::Nothing where {F <: Modia3D.VarFloatType, TimeType}
    @inbounds for obj in tree
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            revolute = mbs.revolute[obj.jointIndex]

            obj.a0 = parent.a0
            z_rel  = revolute.eAxis*revolute.a
            obj.z  = obj.R_rel*(parent.z + z_rel)

        elseif jointKind == PrismaticKind
            prismatic = mbs.prismatic[obj.jointIndex]

            a_rel  = prismatic.eAxis*prismatic.a
            obj.a0 = parent.a0 + parent.R_abs'*(a_rel + cross(parent.z, obj.r_rel))
            obj.z  = parent.z

        elseif jointKind == AbsoluteFreeMotionKind
            freeMotion = mbs.freeMotion[obj.jointIndex]

            obj.a0 = freeMotion.a
            obj.z  = freeMotion.z

        elseif jointKind == FreeMotionKind
            freeMotion = mbs.freeMotion[obj.jointIndex]

            obj.a0 = parent.a0 + parent.R_abs'*(freeMotion.a + cross(parent.z, obj.r_rel))
            obj.z  = parent.R_abs*parent.z + freeMotion.z

        elseif jointKind == FixTranslationKind
            obj.a0 = parent.a0 + parent.R_abs'*(cross(parent.z, obj.r_rel))
            obj.z  = parent.z

        elseif jointKind == FixKind
            obj.a0 = parent.a0 + parent.R_abs'*(cross(parent.z, obj.r_rel))
            obj.z  = obj.R_rel*parent.z

        else
            error("Bug in Modia3D/src/Composition/joints/joints.jl (computeAccelerations!): jointKind = $jointKind is not known.")
        end
    end
    return nothing
end



"""
    computeObject3DForcesTorquesAndGenForces!(mbs::MultibodyData{F,TimeType}, tree::Vector{Object3D{F}}, time)

Traverse the tree of Object3Ds from leaf objects to world, and transform/propagate the forces/torques
from the Object3D to its parent Object3D. Furthermore, store the projection of the force/torque
along the degrees of freedoms of the joints in the joints.

It is assumed that the force/torque variables of every Object3D are initialized (e.g. to zero), including
tree[1].parent.
"""
function computeObject3DForcesTorquesAndGenForces!(mbs::MultibodyData{F,TimeType}, tree::Vector{Object3D{F}}, time)::Nothing where {F <: Modia3D.VarFloatType,TimeType}
    revoluteGenForces   = mbs.revoluteGenForces
    prismaticGenForces  = mbs.prismaticGenForces
    freeMotionGenForces = mbs.freeMotionGenForces
    hiddenGenForces     = mbs.hiddenGenForces

    @inbounds for i = length(tree):-1:1
        obj       = tree[i]
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            revolute  = mbs.revolute[obj.jointIndex]
            parent.f += obj.R_rel'*obj.f
            parent.t += obj.R_rel'*obj.t
            obj.f     = -obj.f
            obj.t     = -obj.t
            revoluteGenForces[obj.jointIndex] = dot(revolute.eAxis,obj.t)

        elseif jointKind == PrismaticKind
            prismatic = mbs.prismatic[obj.jointIndex]
            parent.f += obj.f
            parent.t += obj.t + cross(obj.r_rel, obj.f)
            obj.f     = -obj.f
            obj.t     = -obj.t
            prismaticGenForces[obj.jointIndex] = dot(prismatic.eAxis,obj.f)

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            freeMotion = mbs.freeMotion[obj.jointIndex]
            if freeMotion.hiddenStates
                j = freeMotion.iqdd_hidden
                hiddenGenForces[j  ] = obj.f[1]
                hiddenGenForces[j+1] = obj.f[2]
                hiddenGenForces[j+2] = obj.f[3]
                hiddenGenForces[j+3] = obj.t[1]
                hiddenGenForces[j+4] = obj.t[2]
                hiddenGenForces[j+5] = obj.t[3]
            else
                j = 2*obj.jointIndex
                freeMotionGenForces[j-1] = obj.f
                freeMotionGenForces[j]   = obj.t
            end

        elseif jointKind == FixTranslationKind
            parent.f += obj.f
            parent.t += obj.t + cross(obj.r_rel, obj.f)

        elseif jointKind == FixKind
            f = obj.R_rel'*obj.f
            parent.f += f
            parent.t += obj.R_rel'*obj.t + cross(obj.r_rel, f)

        else
            error("Bug in Modia3D/src/Composition/joints/joints.jl (computeGeneralizedForces!): jointKind = $jointKind is not known.")
        end
    end
    copyto!(mbs.genForces,                           1, revoluteGenForces , 1, length(revoluteGenForces))
    copyto!(mbs.genForces, length(revoluteGenForces)+1, prismaticGenForces, 1, length(prismaticGenForces))
    return nothing
end


"""
    setStatesRevolute!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N}) where {F,TimeType,N}

Copy states of the revolute joints into the corresponding Object3Ds.
"""
function setStatesRevolute!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N == 2*length(mbs.revoluteObjects))
    j = 1
    @inbounds for obj in mbs.revoluteObjects
        revolute     = mbs.revolute[obj.jointIndex]
        revolute.phi = args[j]
        revolute.w   = args[j+1]
        j += 2
    end
    return mbs
end


"""
    setStatesPrismatic!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N}) where {F,N}

Copy states of the prismatic joints into the corresponding Object3Ds.
"""
function setStatesPrismatic!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N == 2*length(mbs.prismaticObjects))
    j = 1
    @inbounds for obj in mbs.prismaticObjects
        prismatic   = mbs.prismatic[obj.jointIndex]
        prismatic.s = args[j]
        prismatic.v = args[j+1]
        j += 2
    end
    return mbs
end


"""
    setStatesFreeMotion!(mbs::MultibodyData{F,TimeType}, args::Vararg{SVector{3,F},N}) where {F,N}

Copy states of the free motion joints into the corresponding Object3Ds.
"""
function setStatesFreeMotion!(mbs::MultibodyData{F,TimeType}, args::Vararg{SVector{3,F},N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N== 4*length(mbs.freeMotionObjects))
    j = 1
    @inbounds for obj in mbs.freeMotionObjects
        freeMotion     = mbs.freeMotion[obj.jointIndex]
        freeMotion.r   = args[j]
        freeMotion.v   = args[j+1]
        freeMotion.rot = args[j+2]
        freeMotion.w   = args[j+3]
        j += 4
    end
    return mbs
end


"""
    setStatesFreeMotion_isrot123!(mbs::MultibodyData{F,TimeType}, args::Vararg{Bool,N})

Copy isrot123 of the free motion joints into the corresponding Object3Ds.
"""
function setStatesFreeMotion_isrot123!(mbs::MultibodyData{F,TimeType}, args::Vararg{Bool,N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N == length(mbs.freeMotionObjects))
    @inbounds for (i,obj) in enumerate(mbs.freeMotionObjects)
        mbs.freeMotion[obj.jointIndex].isrot123 = args[i]
    end
    return mbs
end


singularitySafetyMargin(ang) = abs(rem2pi(ang, RoundNearest)) - 1.5  # is negative/positive in valid/singular angle range


"""
    change_rotSequence!(instantiatedModel, freeMotion::FreeMotion, x::AbstractVector, x_segmented::AbstractVector)

Change rotation sequence of `freeMotion.rot` from `x-axis, y-axis, z-axis` to `x-axis, z-axis, y-axis` or visa versa:

- If `freeMotion.isrot123 = true` , set `freeMotion.isrot123 = false` and `x[..] = x_segmented[..] = rot132fromR(Rfromrot123(freeMotion.rot))`

- If `freeMotion.isrot123 = false`, set `freeMotion.isrot123 = true`  and `x[..] = x_segmented[..] = rot123fromR(Rfromrot132(freeMotion.rot))`
"""
function change_rotSequence!(m::Modia.InstantiatedModel, freeMotion::FreeMotion, x::AbstractVector, x_segmented::AbstractVector)::Nothing
    if freeMotion.isrot123
        freeMotion.rot      = rot132fromR(Rfromrot123(freeMotion.rot))
        freeMotion.isrot123 = false
    else
        freeMotion.rot      = rot123fromR(Rfromrot132(freeMotion.rot))
        freeMotion.isrot123 = true
    end
    Modia.copy_w_segmented_value_to_result(m, freeMotion.iextra_isrot123, freeMotion.isrot123)

    # Change x-vector
    startIndex = freeMotion.ix_segmented_rot
    x_segmented[startIndex  ] = freeMotion.rot[1]
    x_segmented[startIndex+1] = freeMotion.rot[2]
    x_segmented[startIndex+2] = freeMotion.rot[3]

    startIndex = freeMotion.ix_rot
    x[startIndex]   = freeMotion.rot[1]
    x[startIndex+1] = freeMotion.rot[2]
    x[startIndex+2] = freeMotion.rot[3]

    # Change z-vector
    m.eventHandler.z[freeMotion.iz_rot2]         = freeMotion.rot[2]
    m.eventHandler.zPositive[freeMotion.iz_rot2] = false
    rotation2ZeroCrossing = singularitySafetyMargin(freeMotion.rot[2])
    @assert(rotation2ZeroCrossing <= 0)

    if m.options.logEvents
        println("        ", freeMotion.path * ".rotationXYZ changed to ", freeMotion.isrot123)
        println("        ", freeMotion.path * ".rotation changed to ", freeMotion.rot)
        println("        ", freeMotion.str_rot2 * " (= ", rotation2ZeroCrossing, ") became <= 0")
    end

    return nothing
end


"""
    setStatesHiddenJoints!(instantiatedModel::Modia.InstantiatedModel, mbs::MultibodyData, x)

Copy states from the hidden state vector instantiatedModel.x_segmented to the hidden joints into the corresponding Object3Ds
and copy some state derivatives into instantiatedModel.der_x_segmented.
"""
function setStatesHiddenJoints!(m::Modia.InstantiatedModel{F,TimeType}, mbs::MultibodyData{F,TimeType}, _x)::Nothing where {F,TimeType}
    x_segmented     = m.x_segmented
    der_x_segmented = m.der_x_segmented
    j1::Int = 0
    j2::Int = 0
    j3::Int = 0
    j4::Int = 0
    for obj in mbs.hiddenJointObjects
        # Copy x_segmented states into freeMotion states
        freeMotion = mbs.freeMotion[obj.jointIndex]
        j1 = freeMotion.ix_segmented_r  ;  freeMotion.r   = SVector{3,F}(x_segmented[j1], x_segmented[j1+1], x_segmented[j1+2])
        j2 = freeMotion.ix_segmented_rot;  freeMotion.rot = SVector{3,F}(x_segmented[j2], x_segmented[j2+1], x_segmented[j2+2])
        j3 = freeMotion.ix_segmented_v  ;  freeMotion.v   = SVector{3,F}(x_segmented[j3], x_segmented[j3+1], x_segmented[j3+2])
        j4 = freeMotion.ix_segmented_w  ;  freeMotion.w   = SVector{3,F}(x_segmented[j4], x_segmented[j4+1], x_segmented[j4+2])
        if freeMotion.wResolvedInParent
            freeMotion.w = Modia3D.resolve2(freeMotion.rot, freeMotion.w, rotationXYZ = freeMotion.isrot123)
        end

        # der(r) = v
        der_x_segmented[j1]   = freeMotion.v[1]
        der_x_segmented[j1+1] = freeMotion.v[2]
        der_x_segmented[j1+2] = freeMotion.v[3]

        if Modia.positive(m, freeMotion.iz_rot2, singularitySafetyMargin(freeMotion.rot[2]), freeMotion.str_rot2, nothing)
            # freeMotion.rot[2] mapped to range -pi .. pi is either >= 1.5 or <= -1.5 (so comes close to singular position pi/2 = 1.57...)
            change_rotSequence!(m, freeMotion, _x, x_segmented)
        end

        # der(rot) = J123or132(rot,isrot123) * w
        der_rot = J123or132(freeMotion.rot, freeMotion.isrot123)*freeMotion.w
        der_x_segmented[j2]   = der_rot[1]
        der_x_segmented[j2+1] = der_rot[2]
        der_x_segmented[j2+2] = der_rot[3]
    end
    return nothing
end


"""
    setAccelerationsRevolute!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N})

Copy accelerations of revolute joints into mbs.
"""
function setAccelerationsRevolute!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N}) where {F,TimeType,N}
    @assert(N == length(mbs.revoluteObjects))
    @inbounds for (i,obj) in enumerate(mbs.revoluteObjects)
        mbs.revolute[obj.jointIndex].a = args[i]
    end
    return mbs
end


"""
    setAccelerationsPrismatic!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N})

Copy accelerations of prismatic joints into mbs.
"""
function setAccelerationsPrismatic!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N}) where {F,TimeType,N}
    @assert(N == length(mbs.prismaticObjects))
    @inbounds for (i,obj) in enumerate(mbs.prismaticObjects)
        mbs.prismatic[obj.jointIndex].a = args[i]
    end
    return mbs
end


"""
    setAccelerationsFreeMotion!(mbs, args...)

Copy accelerations of free motion joints into mbs
"""
function setAccelerationsFreeMotion!(mbs::MultibodyData{F,TimeType}, args::Vararg{SVector{3,F},N}) where {F,TimeType,N}
    @assert(N == 2*length(mbs.freeMotionObjects))
    j = 1
    @inbounds for obj in mbs.freeMotionObjects
        freeMotion   = mbs.freeMotion[obj.jointIndex]
        freeMotion.a = args[j]
        freeMotion.z = args[j+1]
        j += 2
    end
    return mbs
end


"""
    setAccelerationsHiddenJoints!(mbs, qdd_hidden)

Copy accelerations qdd_hidden of hidden joints into mbs
"""
function setAccelerationsHiddenJoints!(mbs::MultibodyData{F,TimeType}, qdd_hidden::Vector{F})::Nothing where {F,TimeType}
    for obj in mbs.hiddenJointObjects
        freeMotion = mbs.freeMotion[obj.jointIndex]
        j = freeMotion.iqdd_hidden
        freeMotion.a = SVector{3,F}(qdd_hidden[j]  , qdd_hidden[j+1], qdd_hidden[j+2])
        freeMotion.z = SVector{3,F}(qdd_hidden[j+3], qdd_hidden[j+4], qdd_hidden[j+5])
    end
    return nothing
end


"""
    setHiddenStatesDerivatives!(instantiatedModel, mbs, genForces)

Copy derivatives of hidden states to instantiatedModel.der_x_segmented
"""
function setHiddenStatesDerivatives!(m::Modia.InstantiatedModel{F,TimeType}, mbs::MultibodyData{F,TimeType}, genForces::Vector{F})::Nothing where {F,TimeType}
    for obj in mbs.hiddenJointObjects
        freeMotion = mbs.freeMotion[obj.jointIndex]

        j1 = freeMotion.ix_segmented_v
        m.der_x_segmented[j1  ] = freeMotion.a[1]
        m.der_x_segmented[j1+1] = freeMotion.a[2]
        m.der_x_segmented[j1+2] = freeMotion.a[3]

        j2 = freeMotion.ix_segmented_w
        z = freeMotion.wResolvedInParent ? Modia3D.resolve1(obj.R_rel, freeMotion.z) : freeMotion.z
        m.der_x_segmented[j2  ] = z[1]
        m.der_x_segmented[j2+1] = z[2]
        m.der_x_segmented[j2+2] = z[3]

        #println("   j1=$j1, j2=$j2, freeMotion.a[2] = ", freeMotion.a[2], ", m.der_x_segmented = ", m.der_x_segmented)
    end
    return nothing
end


"""
    residuals = getGenForcesFreeMotion(mbs::MultibodyData{F,TimeType}, genForces::Vector{F})

Return generalized forces of free motion joints as Vector to be used for residuals vector.
"""
function getGenForcesFreeMotion(mbs::MultibodyData{F,TimeType}, genForces::Vector{F})::Vector{F} where {F,TimeType}
    res = mbs.freeMotionResiduals
    j = 1
    @assert(length(res) == 3*length(mbs.freeMotionGenForces))
    @inbounds for vec in mbs.freeMotionGenForces
        res[j]   = vec[1]
        res[j+1] = vec[2]
        res[j+2] = vec[3]
        j += 3
    end
    return res
end


"""
    residuals = getGenForcesHiddenJoints(mbs::MultibodyData{F,TimeType}, genForces::Vector{F}, qdd_hidden::Vector{F})

Return generalized forces of hidden joints as Vector to be used for residuals vector.
"""
getGenForcesHiddenJoints(mbs, genForces, qdd_hidden) = mbs.hiddenGenForces


# For backwards compatibility (do not use for new models)
function setAngle!(revolute::Revolute, phi::F) where F <: Modia3D.VarFloatType
   obj          = revolute.obj2
   revolute.phi = phi
   obj.R_rel    = Frames.rot_e(revolute.eAxis, phi)
   obj.R_abs    = obj.R_rel*obj.parent.R_abs
   return revolute
end


# For backwards compatibility (do not use for new models)
function setDistance!(prismatic::Prismatic, s::F) where F <: Modia3D.VarFloatType
   obj         = prismatic.obj2
   prismatic.s = s
   obj.r_rel   = prismatic.eAxis*s
   obj.r_abs   = parent.r_abs + parent.R_abs'*obj.r_rel
   return prismatic
end


function jointSpecificTreatment!(oldChild::Object3D{F}, newChild::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if newChild.jointKind == Modia3D.Composition.RevoluteKind
        revertRevoluteKind!(oldChild, newChild)
        return nothing
    elseif newChild.jointKind == Modia3D.Composition.FixKind
        # nothing needs to be done here
        return nothing
    else
        @error(newChild.jointKind, " is not implemented yet")
        return nothing
    end
    return nothing
end
