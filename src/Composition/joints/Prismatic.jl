# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


mutable struct TreeJointPrismatic <: Modia3D.AbstractPrismatic
   _internal::ModiaMath.ComponentInternal  # Data common to assembly component classes
   obj1::Object3D                        # Translation of obj1 to obj2 along axis of obj1
   obj2::Object3D
   canCollide::Bool                        # = true, if collision detection is enabled between the two Object3Ds connected by the Prismatic joint (default = false)
   isDriven::Bool                          # = true, if setDistance!(..) can be called on the joint

   # Definitions special to Prismatic joints
   axis::Int                       #  axis (= 1,2,3 or -1,-2,-3) of translation
   eAxis::ModiaMath.Vector3D   # Unit vector in direction of axis

   s::ModiaMath.RealScalar
   v::ModiaMath.RealScalar
   a::ModiaMath.RealScalar
   f::ModiaMath.RealScalar
   residue::ModiaMath.RealScalar

   P::ModiaMath.RealScalar       # Total power flowing in obj1, obj2, axis computed at communication points (must be zero; = computed with interpolated values of x,derx)

   flange::PrismaticFlange

   TreeJointPrismatic(_internal, obj1, obj2, canCollide, isDriven, axis, eAxis) = new(_internal, obj1, obj2, canCollide, isDriven, axis, eAxis)
end


mutable struct CutJointPrismatic <: Modia3D.AbstractPrismatic
   _internal::ModiaMath.ComponentInternal  # Data common to assembly component classes
   obj1::Object3D                                # Translation of obj1 to obj2 along axis of obj1
   obj2::Object3D

   # Definitions special to Prismatic joints
   visited::Bool                                   # = true, if already visited when traversing the spanning tree

   # Definitions special to Prismatic joints
   axis::Int                          # axis (= 1,2,3, -1,-2,-3) of translation
   eAxis::ModiaMath.Vector3D      # Unit vector in direction of axis
   e_lambda1::ModiaMath.Vector3D  # Unit vector in direction of lambda1
   e_lambda2::ModiaMath.Vector3D  # Unit vector in direction of lambda2

   lambda1::ModiaMath.RealScalar
   lambda2::ModiaMath.RealScalar
   mue1::ModiaMath.RealScalar
   mue2::ModiaMath.RealScalar
   residue1::ModiaMath.RealScalar
   residue2::ModiaMath.RealScalar
   residue1d::ModiaMath.RealScalar
   residue2d::ModiaMath.RealScalar

   CutJointPrismatic(_internal, obj1, obj2, visited, axis, eAxis, e_lambda1, e_lambda2) = new(_internal, obj1, obj2, visited, axis, eAxis, e_lambda1, e_lambda2)
end


function Base.show(io::IO, joint::TreeJointPrismatic)
   print(io,"Prismatic(", ModiaMath.fullName(joint.obj1),
                   ", ", ModiaMath.fullName(joint.obj2),
                      ", axis = ", joint.axis,
                      ", s_start = ", joint.s.start, " m)")
end

#=
function JSON.show_json(io::JSON.StructuralContext,
                         s::JSON.CommonSerialization, joint::TreeJointPrismatic)
   JSON.begin_object(io)
      JSON.show_pair(io, s, "obj_a"  , joint.obj1.name)
      JSON.show_pair(io, s, "obj_b"  , joint.obj2.name)
      JSON.show_pair(io, s, "driven"   , joint.driven)
      JSON.show_pair(io, s, "cutJoint" , joint.cutJoint)
      JSON.show_pair(io, s, "axis"     , joint.axis)
      JSON.show_pair(io, s, "s_start", joint.s.start)
   JSON.end_object(io)
end
=#





get_eAxis(axis::Int) = axis== 1 ? ModiaMath.Vector3D( 1.0,  0.0,  0.0) :
                       axis== 2 ? ModiaMath.Vector3D( 0.0,  1.0,  0.0) :
                       axis== 3 ? ModiaMath.Vector3D( 0.0,  0.0,  1.0) : error("Modia3D.Prismatic: axis = ", axis, " but must be 1, 2 or 3.")



"""
    joint = Modia3D.Prismatic(obj1::Object3D, obj2::Object3D;
                              axis=1, s_start=0, v_start=0, canCollide=false)

Return a `joint` object that constrains the movement of `obj2::`[`Object3D`](@ref)
with respect to `obj1::`[`Object3D`](@ref) along
coordinate axis `axis` (`axis = 1,2,3,-1,-2,-3`). The initial position/velocity of `obj2` with respect
to `obj1` along `axis` is `s_start` [m] and `v_start` [m/s], respectively.
If `canCollide=false`, no collision detection will occur between `obj1` and `obj2`
(and `Object3D`s that are directly or indirectly rigidly fixed to `obj1` or `obj2`).

If a `Prismatic` joint *closes a kinematic loop*, then the already present objects must be consistent
to the `Prismatic` joint that is the frames of `obj1` and `obj2` must be *parallel* to each other and
movement of `obj1` along its axis `axis` with `s_start` results in `obj2`. If `s_start=NaN`,
its value is computed in this case.

# Examples
```julia
using Modia3D
import ModiaMath

@assembly FallingBall(;h=1.0) begin
   world  = Object3D()
   sphere = Object3D( Modia3D.Solid(Modia3D.SolidSphere(0.1), "Aluminium") )

   # Constrain sphere movement (initial placement at position [0,h,0])
   prismatic = Modia3D.Prismatic(world, sphere, axis=2, s_start=h)
end

simulationModel = Modia3D.SimulationModel( FallingBall(h=1.5), stopTime=1.0 )
result          = ModiaMath.simulate!(simulationModel)
ModiaMath.plot(result, ("prismatic.s", "prismatic.v"))
```
"""
function Prismatic(obj1::Object3D, obj2::Object3D;
                   axis::Int = 1,
                   s_start::Number = 0.0,
                   v_start::Number = 0.0,
                   canCollide::Bool = false)
   eAxis = get_eAxis(axis)
   rs_start = Float64(s_start)
   rv_start = Float64(v_start)
   (obj_a,obj_b,cutJoint) = attach(obj1, obj2)
   @assert(1 <= abs(axis) <= 3)
   if obj_b === obj2
      axis2 = axis
   else
      axis2 = -axis
      eAxis = -eAxis
   end

   if cutJoint  # Currently only planar cut-joint, without checking that it is planar
      println("... Prismatic joint connecting ", ModiaMath.fullName(obj1), " with ", ModiaMath.fullName(obj2), " is a cut-joint")

      # Check that obj1 and obj2 are parallel to each other
      R_rel = obj_b.R_abs*obj_a.R_abs
      diff1 = norm(R_rel - ModiaMath.NullRotation)
      if diff1 > Basics.neps
         error("\nError from Modia3D.Prismatic(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
               "The coordinate systems of the provided Object3Ds are not parallel to each other.\n",
               "( norm(R_rel - nullRotation()) = ", string(diff1), " > ", Basics.neps, ")")
      end
      s_start2 =

      if isnan(rs_start)
         # Compute distance along axis
         r_rel = obj_b.r_abs - obj_a.r_abs
         diff2 = cross(eAxis, r_rel)
         if diff2 > Basics.neps
            error("\nError from Modia3D.Prismatic(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                  "Moving from obj1 along its axis axis does not arrive at obj2.\n",
                  "( sin( angle(obj2.r_abs - obj1.r_abs, axis(obj1) )))  = ", string(diff2), " > ", Basics.neps, ")")
         end

      else
         r2 = obj_a.r_abs + obj_a.R_abs'*(eAxis*s_start)
         diff3 = obj_b.r_abs - r2

         if diff3 > Basics.neps
            error("\nError from Modia3D.Prismatic(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", ...):\n",
                 "The origins of the provided Object3Ds are not along axis/s_start.\n",
                 "( norm( <obj2 position> - <obj1.position+s_start along axis>) = ", string(diff33), " > ", Basics.neps,")")
         end
      end

      # Define variables of cut-joint
      joint    =  CutJointPrismatic(ModiaMath.ComponentInternal(),obj_a, obj_b, false, axis, e_lambda1, e_lambda2)
      lambda1   = ModiaMath.RealScalar("lambda1"  , joint, unit="N", info="Constraint force in direction1"   , numericType=ModiaMath.LAMBDA, analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      lambda2   = ModiaMath.RealScalar("lambda2"  , joint, unit="N", info="Constraint force in direction2"   , numericType=ModiaMath.LAMBDA, analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      mue1      = ModiaMath.RealScalar("mue1"     , joint,           info="Stabilizing mue in direction1"    , numericType=ModiaMath.MUE   , analysis=ModiaMath.OnlyDynamicAnalysis)
      mue2      = ModiaMath.RealScalar("mue2"     , joint,           info="Stabilizing mue in direction2"    , numericType=ModiaMath.MUE   , analysis=ModiaMath.OnlyDynamicAnalysis)
      residue1  = ModiaMath.RealScalar("residue1" , joint,           info="Position constraint in direction1", numericType=ModiaMath.FC)
      residue2  = ModiaMath.RealScalar("residue2" , joint,           info="Position constraint in direction2", numericType=ModiaMath.FC)
      residue1d = ModiaMath.RealScalar("residue1d", joint,           info="Velocity constraint in direction1", numericType=ModiaMath.FC    , analysis=ModiaMath.OnlyDynamicAnalysis)
      residue2d = ModiaMath.RealScalar("residue2d", joint,           info="Velocity constraint in direction2", numericType=ModiaMath.FC    , analysis=ModiaMath.OnlyDynamicAnalysis)

      # direction1/direction2
      if axis == 1
         e_lambda1 = ModiaMath.Vector3D(0,1,0)
         e_lambda2 = ModiaMath.Vector3D(0,0,1)
      elseif axis == 2
         e_lambda1 = ModiaMath.Vector3D(0,0,1)
         e_lambda2 = ModiaMath.Vector3D(1,0,0)
      else
         e_lambda1 = ModiaMath.Vector3D(1,0,0)
         e_lambda2 = ModiaMath.Vector3D(0,1,0)
      end

      # Add joint to the frames
      push!(obj_a.twoObject3Dobject, joint)
      push!(obj_b.twoObject3Dobject, joint)
      obj_a.hasCutJoint = true
      obj_b.hasCutJoint = true
   else
      # Joint in the spanning tree
      if isnan(rs_start)
         error("\nerror from Modia3D.Prismatic(", ModiaMath.fullName(obj1), ", ", ModiaMath.fullName(obj2), ", s_start=NaN):\n",
              "Argument s_start = NaN (Not-a-Number). However, this is not allowed if the joint is in the spanning tree")
      end
      joint   = TreeJointPrismatic(ModiaMath.ComponentInternal(), obj_a, obj_b, canCollide, false, axis, eAxis)
      s       = ModiaMath.RealScalar(:s      , joint, unit="m"    , start=rs_start, fixed=true, info="Relative distance along axis         "    , numericType=ModiaMath.XD_EXP)
      v       = ModiaMath.RealScalar(:v      , joint, unit="m/s"  , start=rv_start, fixed=true, info="=der(s): Relative velocity along axis"    , numericType=ModiaMath.XD_IMP    , integral=s, analysis=ModiaMath.OnlyDynamicAnalysis)
      a       = ModiaMath.RealScalar(:a      , joint, unit="m/s^2", start=0.0     ,             info="=der(v): Relative acceleration along axis", numericType=ModiaMath.DER_XD_IMP, integral=v, analysis=ModiaMath.OnlyDynamicAnalysis)
      f       = ModiaMath.RealScalar(:f      , joint, unit="N"    , start=0.0     ,             info="Driving force"                            , numericType=ModiaMath.WR                    , analysis=ModiaMath.QuasiStaticAndDynamicAnalysis)
      residue = ModiaMath.RealScalar(:residue, joint,               start=0.0     ,             info="Prismatic residue"                        , numericType=ModiaMath.FD_IMP                , analysis=ModiaMath.OnlyDynamicAnalysis)

      P       = ModiaMath.RealScalar(:P      , joint, unit="J", info="Total power computed with interpolated x,derx, values (should be zero)", numericType=ModiaMath.WC, analysis=ModiaMath.OnlyDynamicAnalysis)
      joint.flange = PrismaticFlange()

      obj_a.hasChildJoint = true
      obj_b.joint = joint
      obj_b.r_rel = eAxis*rs_start
      obj_b.r_abs = obj_a.r_abs + obj_a.R_abs'*obj_b.r_rel
      obj_b.R_rel = ModiaMath.NullRotation
      obj_b.R_abs = obj_a.R_abs
   end
   return joint
end


function driveJoint!(joint::TreeJointPrismatic)
   joint.isDriven = true
   joint.s.numericType = ModiaMath.WR
   joint.v.numericType = ModiaMath.WR
   joint.a.numericType = ModiaMath.WR
end


function setDistance!(joint::TreeJointPrismatic, s::Float64)
   @assert( joint.isDriven )
   joint.s.value = s
   joint.obj2.r_rel = joint.eAxis*s
   return joint
end


function computeKinematics!(joint::TreeJointPrismatic, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D = obj.parent

   obj.r_rel = joint.eAxis*joint.s.value
   obj.r_abs = parent.r_abs + parent.R_abs'*obj.r_rel
   obj.R_abs = parent.R_abs

   if analysis == ModiaMath.DynamicAnalysis
      dynamics::Object3Ddynamics       = obj.dynamics
      parentDynamics::Object3Ddynamics = parent.dynamics

      v_rel       = joint.eAxis*joint.v.value
      a_rel       = joint.eAxis*joint.a.value
      dynamics.v0 = parentDynamics.v0 + parent.R_abs'*(v_rel + cross(parentDynamics.w, obj.r_rel))
      dynamics.a0 = parentDynamics.a0 + parent.R_abs'*(a_rel + cross(parentDynamics.z, obj.r_rel) +
                    cross(parentDynamics.w, v_rel + cross(parentDynamics.w, obj.r_rel)) )
      dynamics.w  = parentDynamics.w
      dynamics.z  = parentDynamics.z
   end
   return nothing
end


function computeForceTorqueAndResidue!(joint::TreeJointPrismatic, obj::Object3D, analysis::ModiaMath.AnalysisType, time::Float64)::NOTHING
   parent::Object3D                 = obj.parent
   dynamics::Object3Ddynamics       = obj.dynamics
   parentDynamics::Object3Ddynamics = parent.dynamics
   parentDynamics.f  += dynamics.f
   parentDynamics.t  += dynamics.t + cross(obj.r_rel, dynamics.f)
   dynamics.f         = -dynamics.f
   dynamics.t         = -dynamics.t

   joint.residue.value = -joint.f.value + dot(joint.eAxis,dynamics.f)

   # For checking: compute total power
   joint.P.value = dot(parentDynamics.f, joint.obj1.R_abs*parentDynamics.v0) +
                   dot(dynamics.f      , joint.obj2.R_abs*dynamics.v0) +
                   dot(parentDynamics.t, parentDynamics.w) +
                   dot(dynamics.t      , dynamics.w) -
                   joint.f.value*joint.v.value
   return nothing
end


function computePositionResidues!(joint::CutJointPrismatic, time::Float64)
   r_rel = joint.obj1.R_abs*(joint.obj2.r_abs - joint.obj1.r_abs)
   joint.residue_x.value = dot(joint.e_lambda1, r_rel)
   joint.residue_y.value = dot(joint.e_lambda2, r_rel)
   return nothing
end


function computeVelocityResidues!(joint::CutJointPrismatic, time::Float64)
   v_rel = joint.obj1.R_abs*(joint.obj2.dynamics.v0 - joint.obj1.dynamics.v0)
   joint.residue_vx.value = dot(joint.e_lambda1, v_rel)
   joint.residue_vy.value = dot(joint.e_lambda2, v_rel)
   return nothing
end


function computeCutForcesAndToques!(joint::CutJointPrismatic, time::Float64)
   dynamics1.f += joint.e_lambda1*lambda1 + joint.e_lambda2*lambda2
   dynamics2.f += dynamics.f
end


#-----------------------------------------------------------


function assignPrismaticFlange(flange::PFlange, prismFlange::PrismaticFlange, joint::Modia3D.AbstractPrismatic, assembly::Modia3D.Composition.AssemblyInternal)
  names = fieldnames(typeof(flange))
  for val in names # loops over s,v,a,f
    flange_val    = getfield(flange, val)
    prismFlange_val = getfield(prismFlange,val)
    joint_val     = getfield(joint,val)
    if causalitiesAreOk(flange_val, joint_val)
        setFlangeVariable!(flange_val, prismFlange_val)
        setJointVariable!(joint_val, prismFlange_val)
        addVariableToArray(flange_val, joint_val, assembly)
    else
      return false
    end
  end
  return true
end


function connect(torqueObj::Modia3D.AbstractForceAdaptor, joint::Modia3D.AbstractPrismatic)
  actualAssembly = torqueObj._internal.within._internal

  flangeTorque  = torqueObj.flange
  flangeJoint   = joint.flange

  if !assignPrismaticFlange(flangeTorque, flangeJoint, joint, actualAssembly) # !assignRevoluteFlangeAndSignal(signal, flangeSig, joint, flangeJoint, assembly)
    error("The causalities are different. Please, check all flanges which are connected with joint = ", joint ,".")
  end
  if !in(torqueObj.forceElement, actualAssembly.uniqueForceTorques)
    push!(actualAssembly.uniqueForceTorques, torqueObj.forceElement)
  end
  if !isSInput(flangeJoint) && isFInput(flangeJoint)
    joint.isDriven = false
  elseif isSInput(flangeJoint) && !isFInput(flangeJoint)
    driveJoint!(joint)
  end
end
connect(joint::Modia3D.AbstractPrismatic, torqueObj::Modia3D.AbstractForceAdaptor) = connect(torqueObj::Modia3D.AbstractForceAdaptor, joint::Modia3D.AbstractPrismatic)



function connect(signal::Modia3D.AbstractSignalAdaptor, joint::Modia3D.AbstractPrismatic)
    actualAssembly = signal._internal.within._internal

    flangeSig   = signal.flange
    flangeJoint = joint.flange

    if !assignPrismaticFlange(flangeSig, flangeJoint, joint, actualAssembly) # !assignRevoluteFlangeAndSignal(signal, flangeSig, joint, flangeJoint, assembly)
      error("The causalities are different. Please, check all flanges which are connected with joint = ", joint ,".")
    end
    if !in(signal.signal, actualAssembly.uniqueSignals)
      push!(actualAssembly.uniqueSignals, signal.signal)
    end
    if !isSInput(flangeJoint) && isFInput(flangeJoint)
      joint.isDriven = false
    elseif isSInput(flangeJoint) && !isFInput(flangeJoint)
      driveJoint!(joint)
    end
end
connect(joint::Modia3D.AbstractPrismatic, signal::Modia3D.AbstractSignalAdaptor) = connect(signal::Modia3D.AbstractSignalAdaptor, joint::Modia3D.AbstractPrismatic)













































