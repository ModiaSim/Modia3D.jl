# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


# Dummy data
struct EmptyObject3Ddata <: Modia3D.AbstractObject3Ddata
end
const emptyObject3DData = EmptyObject3Ddata()


struct EmptyTwoObject3DObject <: Modia3D.AbstractTwoObject3DObject
end
const emptyTwoObject3DObject = EmptyTwoObject3DObject()


# FixedJoint is used if an Object3D is rigidly fixed to its parent Object3D
struct FixedJoint <: Modia3D.AbstractJoint
end
fixedJoint = FixedJoint()



# FreeMotion is used if an Object3D is moving freely with respect to its parent Object3D
mutable struct FreeMotion <: Modia3D.AbstractJoint 
   canCollide::Bool                        # = true, if collision detection is enabled between the two Object3Ds connected by the FreeMotion joint (default = true) 
   isDriven::Bool                          # = true, if joint is driven; = false, if generalized coordinates are used as states

   # Definitions special to FreeMotions joints
   r::ModiaMath.RealSVector3
   v::ModiaMath.RealSVector3
   a::ModiaMath.RealSVector3
   q::ModiaMath.RealSVector{4}
   derq::ModiaMath.RealSVector{4}
   w::ModiaMath.RealSVector3
   z::ModiaMath.RealSVector3

   residue_w::ModiaMath.RealSVector3
   residue_f::ModiaMath.RealSVector3
   residue_t::ModiaMath.RealSVector3
   residue_q::ModiaMath.RealScalar

   function FreeMotion(obj::Modia3D.AbstractAssemblyComponent;
                       r_start::AbstractVector = ModiaMath.ZeroVector3D,
                       v_start::AbstractVector = ModiaMath.ZeroVector3D,
                       q_start::AbstractVector = ModiaMath.NullQuaternion,
                       w_start::AbstractVector = ModiaMath.ZeroVector3D)

      r    = ModiaMath.RealSVector3(  :r   , numericType=ModiaMath.XD_EXP                ,             unit="m"    , start=r_start, fixed=true , analysis=ModiaMath.OnlyDynamicAnalysis, info="Relative position vector from origin of obj 1 to origin of obj 2, resolved in obj1")
      v    = ModiaMath.RealSVector3(  :v   , numericType=ModiaMath.XD_IMP                , integral=r, unit="m/s"  , start=v_start, fixed=true , analysis=ModiaMath.OnlyDynamicAnalysis, info="der(r): relative velocity of origin of obj 2 with respect to origin of obj 1, resolved in obj 1")
      a    = ModiaMath.RealSVector3(  :a   , numericType=ModiaMath.DER_XD_IMP            , integral=v, unit="m/s^2",                                                                     info="der(v): relative acceleration of origin of obj 2 with respect to origin of obj 1, resolved in obj 1")   
     
      q    = ModiaMath.RealSVector{4}(:q   , numericType=ModiaMath.XD_IMP                ,               start=q_start, fixed=false,                                         info="Relative quaternion to rotate obj 1 into obj 2")
      derq = ModiaMath.RealSVector{4}(:derq, numericType=ModiaMath.DER_XD_IMP, integral=q, unit="1/s",                               analysis=ModiaMath.OnlyDynamicAnalysis, info="der(q)")
      w    = ModiaMath.RealSVector3(  :w   , numericType=ModiaMath.XD_IMP                , unit="rad/s", start=w_start, fixed=true , analysis=ModiaMath.OnlyDynamicAnalysis, info="Relative angular velocity of obj 2 with respect to obj 1, resolved in obj 2")
      z    = ModiaMath.RealSVector3(  :z   , numericType=ModiaMath.DER_XD_IMP, integral=w, unit="rad/s^2",                           analysis=ModiaMath.OnlyDynamicAnalysis, info="der(w): Relative angular acceleration of obj 2 with respect to obj 1, resolved in obj 2")

      residue_w = ModiaMath.RealSVector3(:residue_w, numericType=ModiaMath.FD_IMP, analysis=ModiaMath.OnlyDynamicAnalysis, info="Angular velocity residue")
      residue_f = ModiaMath.RealSVector3(:residue_f, numericType=ModiaMath.FD_IMP, analysis=ModiaMath.OnlyDynamicAnalysis, info="Momentum equation residue")
      residue_t = ModiaMath.RealSVector3(:residue_t, numericType=ModiaMath.FD_IMP, analysis=ModiaMath.OnlyDynamicAnalysis, info="Angular momentum equation residue")
      residue_q = ModiaMath.RealScalar(  :residue_q, numericType=ModiaMath.FC    ,                                         info="Quaternion constraint residue")

      joint = new(true, false, r, v, a, q, derq, w, z, residue_w, residue_f, residue_t, residue_q) 
      joint.r._internal.within = obj
      joint.v._internal.within = obj
      joint.a._internal.within = obj
      joint.q._internal.within = obj
      joint.derq._internal.within = obj
      joint.w._internal.within = obj
      joint.z._internal.within = obj
      joint.residue_w._internal.within = obj
      joint.residue_f._internal.within = obj
      joint.residue_t._internal.within = obj
      joint.residue_q._internal.within = obj

      return joint
   end
end



#= For index reduction, a vector of the following struct is needed
mutable struct Object3DAccelerationLevel
   z_rel::SVector{3,Float64}  # Relative angular acceleration of Object3D with respect to parent frame, resolved in Object3D
   a0::SVector{3,Float64}     # Absolute acceleration of Object3D origin, resolved in world frame (= der(v0))
   z::SVector{3,Float64}      # Absolute angular acceleration of Object3D, resolved in Object3D
   f::SVector{3,Float64}      # Cut-force resolved in Object3D
   t::SVector{3,Float64}      # Cut-torque resolved in Object3D
   Object3DAccelerationLevel() = new()
end

   A::Vector{Object3DAccelerationLevel}  # A[1]: Derivative of a0/z + forces/torques at Object3D origin
                                      # A[i]: = der(A[i-1])

=#


mutable struct Object3Ddynamics
   v0::SVector{3,Float64}     # Absolute velocity of Object3D origin, resolved in world frame (= der(r_abs))
   w::SVector{3,Float64}      # Absolute angular velocity of Object3D, resolved in Object3D

   a0::SVector{3,Float64}     # Absolute acceleration of Object3D origin, resolved in world frame (= der(v0))
   z::SVector{3,Float64}      # Absolute angular acceleration of Object3D, resolved in Object3D

   f::SVector{3,Float64}      # Cut-force resolved in Object3D
   t::SVector{3,Float64}      # Cut-torque resolved in Object3D

   Object3Ddynamics() = new(ModiaMath.ZeroVector3D, ModiaMath.ZeroVector3D,
                            ModiaMath.ZeroVector3D, ModiaMath.ZeroVector3D,
                            ModiaMath.ZeroVector3D, ModiaMath.ZeroVector3D)
end


"""
    obj1 = Object3D([data]; visualizeFrame=Modia3D.Inherited)
    obj2 = Object3D(parent [, data]; fixed=true, r=zeros(3), R=nothing, q=nothing, 
                    v_start=zeros(3), w_start=zeros(3), 
                    visualizeFrame=Modia3D.Inherited)

Generate a new Object3D object, that is a coordinate system (= frame) with associated data.
If `parent` is present, the Object3D is defined relatively
to the parent Object3D. If `parent` is not present, the Object3D is either a reference object
(such as the world-object), or the object is connected later with a joint to another 
Object3D. If `fixed=true`, the object is rigidly connect to its parent; otherwise
it is moving freely relative to its parent (mathematically described by quaternions).

Note, there are many convenience functions in ModiaMath.Frames to generate a
ModiaMath.RotationMatrix `R` or a ModiaMath.Quaternion `q`.


# Arguments
- `data::Modia3D.AbstractObject3Ddata`: Optional data associated with Object3D.

- `parent::Object3D`: Parent object.

- `fixed::Bool`:
   - `fixed = true`, if the Object3D is fixed relatively to its parent Object3D at position `r,R,q`.
     It is best to provide the rotation information via `R` in this case. 
   - `fixed = false`, if Object3D can move freely relatively to its parent Object3D and is 
      initially placed at `r,R,q`. The movement is internally described with Quaternion vector `q`.
      Therefore, it is best to provide the rotation information via `q` in this case.

- `r::AbstractVector`: Initial relative position vector from frame of parent object to 
   origin of frame object, resolved in parent frame.

- `R::Union{ModiaMath.RotationMatrix,NOTHING}`: Initial rotation matrix defining the rotation 
   from frame of parent object to frame of Object3D. If both `R = nothing` and `q = nothing`,
   a null rotation is defined.

- `q::Union{ModiaMath.Quaternion,NOTHING}`: Initial quaternion defining the rotation
   from frame of parent object to frame of Object3D. If both `R = nothing` and `q = nothing`,
   a null rotation is defined.

- `v_start::AbstractVector`: If `fixed=false`, initial velocity of the origin of Object3D with respect to
  parent, resolved in parent frame.

- `w_start::AbstractVector`: If `fixed=false`, initial angular velocity of Object3D with respect to
  parent, resolved in Object3D.

- `visualizeFrame::Union{Bool,Modia3D.Ternary}`: Coordinate system of Object3D is always (= true),
  or never (= false) visualized, or it is visualized if defined in SceneOptions(...)
  (= Modia3D.Inherited).

# Examples
```julia
using Modia3D

# Define assembly
@assembly MyAssembly begin
   world = Object3D()

   # Frame fixed in world
   frame1 = Object3D(world; r=[0.1, 0.2, 0.3])

   # Frame moving relatively to frame1
   r2     = [0.2, 0.2, 0.3]
   frame2 = Object3D(frame1; fixed=false, r=r2)

   # Frame moving relatively to world
   frame3 = Object3D(world; fixed=false, r=-r2)
end
Modia3D.visualizeAssembly!(MyAssembly())
```
"""
mutable struct Object3D <: Modia3D.AbstractAssemblyComponent
   _internal::ModiaMath.ComponentInternal       # Data common to assembly component classes

   # Tree connection structure of Object3D (for efficient processing of Object3Ds)
   parent::Object3D                             # Parent Object3D (if parent===Object3D, no parent is yet defined)
   children::Vector{Object3D}                   # All Object3Ds, where Object3D is the parent

   # Relative position of Object3D with respect to parent Object3D
   joint::Modia3D.AbstractJoint                 # Object that is used to compute r_rel and R_rel
   isDrivenRot::Bool                            # frame or joint is driven from outside
   isDrivenTrans::Bool                          # frame or joint is driven from outside
   isDriven::Bool                               # frame or joint is driven from outside
   r_rel::SVector{3,Float64}                    # Relative position vector from frame of parent Object3D to origin of Object3D frame, resolved in parent frame
                                                # (if parent===Object3D, r_rel=ModiaMath.ZeroVector3D)
   R_rel::SMatrix{3,3,Float64,9}                # Rotation matrix from frame of parent Object3D to Object3D frame.
                                                # (if parent===Object3D, R_rel=ModiaMath.NullRotation)
   # Absolute position of frame
   r_abs::SVector{3,Float64}                    # Absolute position vector from origin of world frame to origin of Object3D frame, resolved in world frame
   R_abs::SMatrix{3,3,Float64,9}                # Absolute rotation matrix from world frame to Object3D frame

   # Additional information associated with Object3D
   data::Modia3D.AbstractObject3Ddata                      # Optional data associated with Object3D
   twoObject3Dobject::Vector{Modia3D.AbstractTwoObject3DObject}  # Optional AbstractTwoObject3DObject object associated with Object3D
   visualizeFrame::Modia3D.Ternary                         # = True     : Coordinate system of Object3D is always visualized
                                                           # = False    : Coordinate system of Object3D is never visualized
                                                           # = Inherited: Coordinate system of Object3D is visualized, if SceneOptions(visualizeFrames=true)
   visualizationFrame::Union{Object3D,NOTHING}                # If to be visualized, the Object3D holding the coordinate system.

   # Data for dynamic simulation
   dynamics::Union{Object3Ddynamics, NOTHING}   # Data for dynamic simulation


   # Object3D constructor without parent
   function Object3D(data::Modia3D.AbstractObject3Ddata = emptyObject3DData;
                     visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited)::Object3D
      obj                    = new(ModiaMath.ComponentInternal())
      obj.parent             = obj
      obj.children           = Vector{Object3D}[]
      obj.joint              = fixedJoint
      obj.isDrivenRot        = false
      obj.isDrivenTrans      = false
      obj.isDriven           = false
      obj.r_rel              = ModiaMath.ZeroVector3D
      obj.R_rel              = ModiaMath.NullRotation
      obj.r_abs              = ModiaMath.ZeroVector3D
      obj.R_abs              = ModiaMath.NullRotation
      obj.data               = data
      obj.twoObject3Dobject  = Modia3D.AbstractTwoObject3DObject[]
      obj.visualizeFrame     = typeof(visualizeFrame) == Modia3D.Ternary ? visualizeFrame : (visualizeFrame ? Modia3D.True : Modia3D.False)
      obj.visualizationFrame = nothing
      obj.dynamics           = nothing
      return obj
   end


   # Object3D constructor with parent
   function Object3D(parent::Object3D,
                     data::Modia3D.AbstractObject3Ddata = emptyObject3DData;
                     fixed::Bool = true,
                     r::AbstractVector = ModiaMath.ZeroVector3D,
                     R::Union{ModiaMath.RotationMatrix,NOTHING} = nothing,
                     q::Union{ModiaMath.Quaternion,NOTHING} = nothing,
                     v_start::AbstractVector = ModiaMath.ZeroVector3D,
                     w_start::AbstractVector = ModiaMath.ZeroVector3D,
                     visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited)::Object3D

      if typeof(R) != NOTHING && typeof(q) != NOTHING
         error("Modia3D.Object3D: either R or q must be nothing but both have a value.")
      end
      if typeof(R) != NOTHING
         ModiaMath.assertRotationMatrix(R)
      elseif typeof(q) != NOTHING
         ModiaMath.assertQuaternion(q)
      end

      r_rel = SVector{3,Float64}(r)
      R_rel = typeof(R) != NOTHING ? R : (typeof(q) != NOTHING ? ModiaMath.from_q(q) : ModiaMath.NullRotation)
      r_abs = parent.r_abs + r_rel
      R_abs = R_rel*parent.R_abs

      visualizeFrame2 = typeof(visualizeFrame) == Modia3D.Ternary ? visualizeFrame : (visualizeFrame ? Modia3D.True : Modia3D.False)

      obj = new(ModiaMath.ComponentInternal(), parent, Vector{Object3D}[], fixedJoint,
                false, false, false, r_rel, R_rel, r_abs, R_abs, data,
                Modia3D.AbstractTwoObject3DObject[], visualizeFrame2, nothing, nothing)

      if !fixed
         obj.joint = FreeMotion(obj; r_start = r_rel, 
                                q_start = typeof(R) != NOTHING ? ModiaMath.from_R(R) : 
                                          typeof(q) != NOTHING ? q : ModiaMath.NullQuaternion,
                                v_start = SVector{3,Float64}(v_start),
                                w_start = SVector{3,Float64}(w_start) )
      end

      push!(parent.children, obj)
      return obj
   end


   # Constructor used only for internal purposes (not to be directly used by the user)
   Object3D(_internal::ModiaMath.ComponentInternal,
            parent::Object3D, children::Vector{Object3D}, 
            joint::Modia3D.AbstractJoint, r_rel::SVector{3,Float64},
            R_rel::SMatrix{3,3,Float64,9}, r_abs::SVector{3,Float64},
            R_abs::SMatrix{3,3,Float64,9}, data::Modia3D.AbstractObject3Ddata,
            twoObject3Dobject::Vector{Modia3D.AbstractTwoObject3DObject},
            visualizeFrame::Modia3D.Ternary) =
      new(_internal, parent, children, joint, false, false, false, r_rel,
          R_rel, r_abs, R_abs, data, twoObject3Dobject, visualizeFrame, nothing, nothing)
end



# Object3D constructor without parent and with convex decomposition of the solid
function Object3D(data::Solids.SolidWithConvexDecomposition;
                  visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited)::Object3D
   obj      = Object3D(data.solid; visualizeFrame=visualizeFrame)
   elements = data.convexDecomposition
   names    = data.names
   for i in eachindex(elements)
      obj_element = Object3D(obj, elements[i]; visualizeFrame=Modia3D.False)
      obj_element._internal.within = obj
      obj_element._internal.name   = names[i]
   end
   return obj
end



# Object3D constructor with parent and with convex decomposition of the solid
function Object3D(parent::Object3D,
                  data::Solids.SolidWithConvexDecomposition;
                  fixed::Bool = true,
                  r::AbstractVector = ModiaMath.ZeroVector3D,
                  R::Union{ModiaMath.RotationMatrix,NOTHING} = nothing,
                  q::Union{ModiaMath.Quaternion,NOTHING} = nothing,
                  v_start::AbstractVector = ModiaMath.ZeroVector3D,
                  w_start::AbstractVector = ModiaMath.ZeroVector3D,
                  visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited)::Object3D
   obj      = Object3D(parent, data.solid; fixed=fixed, r=r, R=R, q=q, v_start=v_start, w_start=w_start, visualizeFrame=visualizeFrame)
   elements = data.convexDecomposition
   names    = data.names
   for i in eachindex(elements)
      obj_element = Object3D(obj, elements[i]; visualizeFrame=Modia3D.False)
      obj_element._internal.within = obj
      obj_element._internal.name   = Symbol(names[i])
   end
   return obj;
end

function driveJoint!(frame::Object3D)::NOTHING
   frame.isDriven = true
   return nothing
end


# Make a direct copy of the first level of frame (only used internally in Modia3D)
copyObject3D(obj::Object3D, data::Modia3D.AbstractObject3Ddata) = Object3D(obj._internal, obj.parent, obj.children, obj.joint,
                                                                           obj.r_rel, obj.R_rel, obj.r_abs, obj.R_abs, data,
                                                                           obj.twoObject3Dobject, obj.visualizeFrame)

# Inquire properties of a Object3D
hasParent(            obj::Object3D) = !(obj.parent === obj)
hasNoParent(          obj::Object3D) =   obj.parent === obj
hasChildren(          obj::Object3D) = length(obj.children) > 0
hasNoChildren(        obj::Object3D) = length(obj.children) == 0
isWorld(              obj::Object3D) = hasNoParent(obj) && ModiaMath.isInComponent(obj) && typeof(obj._internal.within._internal.scene) == Scene && obj._internal.within._internal.scene.initAnalysis
isNotWorld(           obj::Object3D) = !(isWorld(obj))
isFixed(              obj::Object3D) = typeof(obj.joint) == FixedJoint
isNotFixed(           obj::Object3D) = typeof(obj.joint) != FixedJoint
isFree(               obj::Object3D) = typeof(obj.joint) == FreeMotion
isNotFree(            obj::Object3D) = typeof(obj.joint) != FreeMotion
hasJoint(             obj::Object3D) = isNotFixed(obj) && isNotFree(obj)
hasNoJoint(           obj::Object3D) = isFixed(obj) || isFree(obj)
hasData(              obj::Object3D) = typeof(obj.data) != EmptyObject3Ddata
isCoordinateSystem(   obj::Object3D) = typeof(obj.data) == Graphics.CoordinateSystem
isNotCoordinateSystem(obj::Object3D) = typeof(obj.data) != Graphics.CoordinateSystem


# The following functions need to be defined for every type derived from AbstractObject3Ddata,
# provided the function should return true


isVisible( data::Modia3D.AbstractObject3Ddata, renderer::Modia3D.AbstractRenderer) = false
canCollide(data::Modia3D.AbstractObject3Ddata) = false
hasMass(   data::Modia3D.AbstractObject3Ddata) = false
hasMass(   data::Solids.Solid)                 = typeof(data.massProperties)!= NOTHING


# Inquire properties of an Object3D that depend on the type of frame.data
"""
    isVisible(obj::Object3D, renderer::Modia3D.AbstractRenderer)

returns true, if `obj` is visualized with renderer `renderer`.
"""
isVisible( obj::Object3D, renderer::Modia3D.AbstractRenderer) = isVisible(obj.data, renderer)

function canCollide(obj::Object3D)
  if typeof(obj.data) == Modia3D.Solids.Solid
    return typeof(obj.data.contactMaterial) != NOTHING && typeof(obj.data.geo) != NOTHING
  else
    return false
  end
end
hasMass(obj::Object3D) = hasMass(obj.data)


"""    rootObject3D(frame) - returns the root frame of all parents of frame"""
function rootObject3D(frame::Object3D)::Object3D
   frame1 = frame
   while hasParent(frame1)
      frame1 = frame1.parent
   end
   return frame1
end


"""    removeChild!(frame, child) - Remove child from frame.children"""
function removeChild!(frame::Object3D, child::Object3D)::NOTHING
   children = frame.children
   for i in eachindex(children)
      if children[i] === child
         deleteat!(children,i)
         return nothing
      end
   end
   error("\nError from Modia3D.Composition.removeChild!(", frame.name, ", ", child.name, ")\n",
         child.name, " is not a child of frame ", frame.name)
end



# Print Object3D
#   dataName = Basics.trailingPartOfTypeAsString(frame.data)
function Base.show(io::IO, frame::Object3D)
   commaNeeded = true
   if frame.parent === frame || hasJoint(frame)
      commaNeeded = false
      print(io,"Object3D(")
   else
      print(io,"Object3D(", ModiaMath.fullName(frame.parent))
   end

   if typeof(frame.data) != EmptyObject3Ddata
      if commaNeeded
         print(io,", ")
      end
      typeName = string( typeof(frame.data) )
      ss = split(typeName, ".")

      # override and collapse Modia3D.x.y.z.Type into Modia3D.Type
      if ss[1] == "Modia3D" && length(ss) > 1
         typeName = ss[1] * ss[end]
      end

      print(io, typeName, "(...)")
   end

   if frame.r_rel != ModiaMath.ZeroVector3D
      print(io, "; r_rel = ", frame.r_rel, ")")
   else
      print(io, ")")
   end
   print(io, " # ", ModiaMath.instanceName(frame))
end
