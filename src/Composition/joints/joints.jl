# License for this file: MIT (expat)
# Copyright 2017-2021, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

mutable struct MultibodyData{F <: Modia3D.VarFloatType, TimeType}
    instantiatedModel::Modia.SimulationModel{F,TimeType}

    nqdd::Int                                 # Length of qdd vector
    world::Object3D{F}                        # Pointer to world object
    scene::Scene{F}                           # Pointer to scene
    
    revoluteObjects::Vector{Object3D{F}}      # References to Object3Ds with a Revolute joint
    prismaticObjects::Vector{Object3D{F}}     # References to Object3Ds with a Prismatic joint
    freeMotionObjects::Vector{Object3D{F}}    # References to Object3Ds with a FreeMotion joint
    
    revoluteGenForces::Vector{F}              # = M_r*der(qd_r) + h_r(q,qd,gravity,contact-forces) = <generalized driving torques>
    prismaticGenForces::Vector{F}             # = M_p*der(qd_p) + h_p(q,qd,gravity,contact-forces) = <generalized driving forces>    
    freeMotionGenForces::Vector{SVector{3,F}} # = M_f*der(qd_f) + h_f(q,qd,gravity,contact-forces) = 0
    
    revoluteCache_h::Vector{F}                # = h_r(q,qd,gravity,contact-forces)
    prismaticCache_h::Vector{F}               # = h_p(q,qd,gravity,contact-forces)
    freeMotionCache_h::Vector{SVector{3,F}}   # = h_f(q,qd,gravity,contact-forces)
    
    zStartIndex::Int                          # eventHandler.z[zStartIndex] is first index of crossing function
                                              # (or zero, if enableContactDetection=false)
    nz::Int                                   # Number of used zero crossing functions
    time::TimeType                            # Current time

    freeMotionResiduals::Vector{F}  
    
    # for multibodyAccelerations
    leq::Vector{Modia.LinearEquations{F}}

    MultibodyData{F,TimeType}(instantiatedModel, nqdd, world, scene, 
                              revoluteObjects, prismaticObjects, freeMotionObjects,
                              zStartIndex, nz, time) where {F,TimeType} =
                         new(instantiatedModel, nqdd, world, scene, 
                             revoluteObjects, prismaticObjects, freeMotionObjects,
                             zeros(F, length(revoluteObjects)), zeros(F, length(prismaticObjects)), zeros(SVector{3,F}, 2*length(freeMotionObjects)),
                             zeros(F, length(revoluteObjects)), zeros(F, length(prismaticObjects)), zeros(SVector{3,F}, 2*length(freeMotionObjects)), 
                             zStartIndex, nz, Modia3D.convertAndStripUnit(TimeType, u"s", time), 
                             zeros(F, 2*3*length(freeMotionObjects)),
                             Modia.LinearEquations{F}[])
end

mutable struct MultibodyBuild{F <: Modia3D.VarFloatType, TimeType}
    Model3DPath::String                         # Path of the Model3D(..) command used to define the system, e.g. "a.b.c"
    Model3DSplittedPath::Vector{Symbol}         # Splitted Model3DPath, e.g. [:a, :b, :c]
    revoluteIndices::OrderedDict{String,Int}    # obj.jointIndex = revoluteIndices["a.b.c"] (= order of arguments in setStatesRevolute!(..., args...))
    prismaticIndices::OrderedDict{String,Int}   # obj.jointIndex = prismaticIndices["a.b.c"] (= order of arguments in setStatesPrismatic!(..., args...))
    freeMotionIndices::OrderedDict{String,Int}  # obj.jointIndex = freeMotionIndices["a.b.c"] (= order of arguments in setStatesFreeMotion!(..., args...))
    mbs::Union{MultibodyData{F,TimeType}, Nothing}  
    
    MultibodyBuild{F,TimeType}(Model3DPath::String, Model3DSplittedPath::Vector{Symbol}, revoluteIndices, prismaticIndices, freeMotionIndices) where {F,TimeType} =
                              new(Model3DPath, Model3DSplittedPath, revoluteIndices, prismaticIndices, freeMotionIndices, nothing)    
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


# Next function only for backwards compatibility (do not use for new model)
#computeKinematics!(scene::Scene, joint::Modia3D.AbstractJoint, obj::Object3D, analysis::Modia3D.AnalysisType, time)::Nothing =
#    computeKinematics!(scene, [obj], Float64(time) )


"""
    computePositionsVelocitiesAccelerations!(scene::Scene{F}, tree::Vector{Object3D{F}}, time) where F <: Modia3D.VarFloatType

Compute position, velocity, acceleration variables of the Object3Ds that are connected
in form of a tree. Variable `tree` contains the Object3Ds in a traversal order (e.g. pre-order traversal).
`tree[1]` is the root object. It is assumed that the kinematic
variables of tree[1].parent have a meaningful value.
"""
function computePositionsVelocitiesAccelerations!(scene::Scene{F}, tree::Vector{Object3D{F}}, time)::Nothing where F <: Modia3D.VarFloatType
    @inbounds for obj in tree
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
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
            obj.R_rel = freeMotion.isrot123 ? Rfromrot123(freeMotion.rot) : Rfromrot132(freeMotion.rot)

            obj.r_abs = obj.r_rel
            obj.R_abs = obj.R_rel

            obj.v0 = freeMotion.v
            obj.a0 = freeMotion.a

            obj.w  = freeMotion.w
            obj.z  = freeMotion.z

        elseif jointKind == FreeMotionKind
            freeMotion  = scene.freeMotion[obj.jointIndex]

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
    computeAccelerations!(scene::Scene, tree::Vector{Object3D{F}}, time)

Traverse the tree of Object3Ds from world to leaf objects and compute the generalized accelerations of the 
joints and the accelerations of the Object3Ds. Hereby it is assumed that the generalized position/velocities
of the joints and the position/velocities of bhe Object3Ds are already stored in the Object3Ds/joints.
"""
function computeAccelerations!(scene::Scene{F}, tree::Vector{Object3D{F}}, time)::Nothing where F <: Modia3D.VarFloatType
    @inbounds for obj in tree
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
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
    computeObject3DForcesTorquesAndGenForces!(mbs::MultibodyData{F}, tree::Vector{Object3D{F}}, time)

Traverse the tree of Object3Ds from leaf objects to world, and transform/propagate the forces/torques
from the Object3D to its parent Object3D. Furthermore, store the projection of the force/torque
along the degrees of freedoms of the joints in the joints.

It is assumed that the force/torque variables of every Object3D are initialized (e.g. to zero), including
tree[1].parent.
"""
function computeObject3DForcesTorquesAndGenForces!(mbs::MultibodyData{F}, tree::Vector{Object3D{F}}, time)::Nothing where F <: Modia3D.VarFloatType
    scene = mbs.scene
    revoluteGenForces   = mbs.revoluteGenForces
    prismaticGenForces  = mbs.prismaticGenForces
    freeMotionGenForces = mbs.freeMotionGenForces   
    @inbounds for i = length(tree):-1:1
        obj       = tree[i]
        parent    = obj.parent
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            revolute  = scene.revolute[obj.jointIndex]
            parent.f += obj.R_rel'*obj.f
            parent.t += obj.R_rel'*obj.t
            obj.f     = -obj.f
            obj.t     = -obj.t
            revoluteGenForces[obj.jointIndex] = revolute.posMovement ? obj.t[revolute.posAxis] : -obj.t[revolute.posAxis]

        elseif jointKind == PrismaticKind
            prismatic = scene.prismatic[obj.jointIndex]
            parent.f += obj.f
            parent.t += obj.t + cross(obj.r_rel, obj.f)
            obj.f     = -obj.f
            obj.t     = -obj.t
            prismaticGenForces[obj.jointIndex] = dot(prismatic.eAxis,obj.f)          

        elseif jointKind == AbsoluteFreeMotionKind || jointKind == FreeMotionKind
            j = 2*obj.jointIndex 
            freeMotionGenForces[j-1] = obj.f
            freeMotionGenForces[j]   = obj.t    
            
        elseif jointKind == FixTranslationKind
            parent.f += obj.f
            parent.t += obj.t + cross(obj.r_rel, obj.f)

        elseif jointKind == FixKind
            parent.f += obj.R_rel'*obj.f
            parent.t += obj.R_rel'*obj.t + cross(obj.r_rel, obj.f)
            
        else
            error("Bug in Modia3D/src/Composition/joints/joints.jl (computeGeneralizedForces!): jointKind = $jointKind is not known.")
        end
    end
    return nothing
end


"""
    setStatesRevolute!(mbs::MultibodyData{F}, args::Vararg{F,N}) where {F,N}

Copy states of the revolute joints into the corresponding Object3Ds.
"""
function setStatesRevolute!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N == 2*length(mbs.revoluteObjects))
    scene = mbs.scene
    j = 1
    @inbounds for obj in mbs.revoluteObjects
        revolute     = scene.revolute[obj.jointIndex]
        revolute.phi = args[j]
        revolute.w   = args[j+1]
        j += 2
    end
    return mbs
end


"""
    setStatesPrismatic!(mbs::MultibodyData{F}, args::Vararg{F,N}) where {F,N}

Copy states of the prismatic joints into the corresponding Object3Ds.
"""
function setStatesPrismatic!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N == 2*length(mbs.prismaticObjects))
    scene = mbs.scene
    j = 1
    @inbounds for obj in mbs.prismaticObjects
        prismatic   = scene.prismatic[obj.jointIndex]
        prismatic.s = args[j]
        prismatic.v = args[j+1]
        j += 2
    end
    return mbs
end


"""
    setStatesFreeMotion!(mbs::MultibodyData{F}, args::Vararg{SVector{3,F},N}) where {F,N}

Copy states of the free motion joints into the corresponding Object3Ds.
"""
function setStatesFreeMotion!(mbs::MultibodyData{F,TimeType}, args::Vararg{SVector{3,F},N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N== 4*length(mbs.freeMotionObjects))
    scene = mbs.scene
    j = 1
    @inbounds for obj in mbs.freeMotionObjects
        freeMotion     = scene.freeMotion[obj.jointIndex]
        freeMotion.r   = args[j]
        freeMotion.v   = args[j+1]
        freeMotion.rot = args[j+2]        
        freeMotion.w   = args[j+3]
        j += 4
    end
    return mbs
end


"""
    setStatesFreeMotion_isrot123!(mbs::MultibodyData{F}, args::Vararg{Bool,N})

Copy isrot123 of the free motion joints into the corresponding Object3Ds.
"""
function setStatesFreeMotion_isrot123!(mbs::MultibodyData{F,TimeType}, args::Vararg{Bool,N})::MultibodyData{F,TimeType} where {F,TimeType,N}
    @assert(N == length(mbs.freeMotionObjects))
    scene = mbs.scene
    @inbounds for (i,obj) in enumerate(mbs.freeMotionObjects)
        scene.freeMotion[obj.jointIndex].isrot123 = args[i]
    end
    return mbs
end


"""
    setAccelerationsRevolute!(mbs::MultibodyData{F}, args::Vararg{F,N}) where {F,N}

Copy accelerations of revolute joints into mbs.
"""
function setAccelerationsRevolute!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N}) where {F,TimeType,N}
    @assert(N == length(mbs.revoluteObjects))
    scene = mbs.scene
    @inbounds for (i,obj) in enumerate(mbs.revoluteObjects)
        scene.revolute[obj.jointIndex].a = args[i]
    end
    return mbs
end


"""
    setAccelerationsPrismatic!(mbs::MultibodyData{F}, args::Vararg{F,N}) where {F,N}

Copy accelerations of prismatic joints into mbs.
"""
function setAccelerationsPrismatic!(mbs::MultibodyData{F,TimeType}, args::Vararg{F,N}) where {F,TimeType,N}
    @assert(N == length(mbs.prismaticObjects))
    scene = mbs.scene
    @inbounds for (i,obj) in enumerate(mbs.prismaticObjects)
        scene.prismatic[obj.jointIndex].a = args[i]
    end
    return mbs
end


"""
    setAccelerationsFreeMotion!(mbs, args...)

Copy accelerations of free motion joints into mbs
"""
function setAccelerationsFreeMotion!(mbs::MultibodyData{F,TimeType}, args::Vararg{SVector{3,F},N}) where {F,TimeType,N}
    @assert(N == 2*length(mbs.freeMotionObjects))
    scene = mbs.scene
    j = 1
    @inbounds for obj in mbs.freeMotionObjects
        freeMotion   = scene.freeMotion[obj.jointIndex]
        freeMotion.a = args[j]
        freeMotion.z = args[j+1]
        j += 2
    end
    return mbs
end

"""
    (tau1,tau2,<...>) = getGenForcesRevolute(mbs::MultibodyData{F}, ::Val{N}) where {F <: Modia3D.VarFloatType,N}

Return generalized forces of revolute joints as NTuple.
"""
#getGenForcesRevolute(mbs::MultibodyData{F,TimeType}) where {F,TimeType} = mbs.revoluteGenForces
getGenForcesRevolute(mbs::MultibodyData{F,TimeType}, ::Val{N}) where {F,TimeType,N} = ntuple(i->mbs.revoluteGenForces[i], Val(N))


"""
    (f1,f2,<...>) = getGenForcesPrismatic(mbs::MultibodyData{F}, ::Val{N}) where {F <: Modia3D.VarFloatType,N}

Return generalized forces of prismatic joints as NTuple.
"""
#getGenForcesPrismatic(mbs::MultibodyData{F,TimeType}) where {F,TimeType} = mbs.prismaticGenForces
getGenForcesPrismatic(mbs::MultibodyData{F,TimeType}, ::Val{N}) where {F,TimeType,N} = ntuple(i->mbs.prismaticGenForces[i], Val(N))


"""
    residuals = getGenForcesFreeMotion(mbs::MultibodyData{F}, ::Val{N}) where {F <: Modia3D.VarFloatType,N}

Return generalized forces of free motion joints as Vector to be used for residuals vector.
"""
function getGenForcesFreeMotion(mbs::MultibodyData{F,TimeType})::Vector{F} where {F,TimeType}
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


# For backwards compatibility (do not use for new models)
function setAngle!(revolute::Revolute, phi::F) where F <: Modia3D.VarFloatType
   obj          = revolute.obj2
   revolute.phi = phi
   obj.R_rel    = Frames.rotAxis(revolute.posAxis, revolute.posMovement, phi)
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
