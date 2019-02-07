# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#



#-------------------------------------- Default Renderer -------------------------------

initializeVisualization(renderer::Modia3D.AbstractRenderer, allVisuElements::Vector{Object3D}) = error("No renderer defined.")
visualize!(renderer::Modia3D.AbstractRenderer, time::Float64) = error("No renderer defined.")
closeVisualization(renderer::Modia3D.AbstractRenderer)        = error("No renderer defined.")


#-------------------------------------- Default Contact Detection -------------------------------

"""
    ContactPairs(nzmax, collSuperObjs, noCPairs, dummyObject3D)

Generate a new ContactPairs structure used for communication between the Object3D handler and a ContactDetection handler.

  nzmax::Int: Maximum number of zero-crossing functions that shall be used
- collSuperObjs::Vector{Vector{Modia3D.AbstractObject3Ddata}}: Objects3D that can collide with each other.
  collSuperObjs[i] is a vector of Objects3D that are rigidly fixed to each other.
  Note, for ever Object3D in this vector, the following holds:
  c1i = celement[i] -> canCollide(c1i[j]) = true; c1i[j].geo <: AbstractSolidGeometry
- noCPairs::Vector{Vector{Int}}: Objects3D where contact detection shall be switched off.
  If c2i = noCPairs[i] then c2i[j] > i, and collision detections between collSuperObjs[i][:] and
  collSuperObjs[c2i[j]][:] shall not take place.
- DummyObject3D::Modia3D.AbstractObject3Ddata: A dummy Object3D that can be used in the struct as element of a vector of Object3Ds
  to fill the array with a dummy value of the correct type.
"""
struct ContactPairs
   # Solid shapes used in contact detection (provided by Object3D handler)
   collSuperObjs::Array{Array{Object3D}}
   noCPairs::Array{Array{Int64,1}}
   AABB::Array{Array{Basics.BoundingBox}}
   dummyObject3D::Modia3D.AbstractObject3Ddata         # Dummy Object3D for non-used elements of z-Vector.


   # Dimensions
   ne::Int                                         # length(collSuperObjs)
   nz::Int                                         # length(z)
   allPossibleContactPairsInz::Bool                # = true, if nz == number of all possible contact pairs

   # All vectors below have length nz and are computed by functions selectContactPairs!(...) and getDistances!(...)
   z::Vector{Float64}                              # Vector of zero crossing functions. z[i] < 0.0 if i-th contact pair has penetration

   contactPoint1::Vector{MVector{3,Float64}}       # Absolute position vector to first contact point on contactObj1
   contactPoint2::Vector{MVector{3,Float64}}       # Absolute position vector to second contact point on contactObj2
   contactNormal::Vector{MVector{3,Float64}}       # Unit normal to surface on contactPoint1 (in world frame)

   contactObj1::Vector{Union{Object3D,NOTHING}}
   contactObj2::Vector{Union{Object3D,NOTHING}}

   function ContactPairs(superObjs::Array{SuperObjsRow,1},
                         noCPairs::Array{Array{Int64,1}},
                         AABB::Array{Array{Basics.BoundingBox}},
                         nz_max::Int)
      @assert(length(superObjs) > 0)
      @assert(length(noCPairs) == length(superObjs))
      @assert(nz_max > 0)
      dummyObject3D = Composition.emptyObject3DData

      # Determine the dimension of vector z (<= nzmax, but at most the number of all possible contact point combinations)
      nz = 0

      collSuperObjs = Array{Array{Object3D,1},1}()

      for i_superObj = 1:length(superObjs)
        push!(collSuperObjs, superObjs[i_superObj].superObjCollision.superObj)
        superObj = superObjs[i_superObj].superObjCollision.superObj
        for i_next_superObj = i_superObj+1:length(superObjs)
           if !(i_next_superObj in noCPairs[i_superObj])
             for i_obj = 1:length(superObj)
                for i_nextObj =1:length(superObjs[i_next_superObj].superObjCollision.superObj)
                   nz += 1
                   if nz >=nz_max
                      @goto AfterLoops
      end; end; end; end; end; end

      @label AfterLoops
      if nz <= nz_max
         allPossibleContactPairsInz = true
      else
         allPossibleContactPairsInz = false
         nz = nz_max
      end
#=
      for a in collSuperObjs
         println(" ")
         for b in a
            println(ModiaMath.fullName(b))
         end
      end

      println("nz = $nz , nz_max = $nz_max")
=#
      # Allocate storage
      z = fill(42.0, nz)
      defaultPoint   = MVector{3,Float64}(0.0,0.0,0.0)
      contactPoint1  = [defaultPoint for i = 1:nz]
      contactPoint2  = [defaultPoint for i = 1:nz]
      contactNormal  = [defaultPoint for i = 1:nz]

      @static if VERSION >= v"0.7.0-DEV.2005"
          contactObj1    = Vector{Union{Object3D,NOTHING}}(nothing,nz)
          contactObj2    = Vector{Union{Object3D,NOTHING}}(nothing,nz)
      else
          contactObj1    = Vector{Union{Object3D,NOTHING}}(nz)
          contactObj2    = Vector{Union{Object3D,NOTHING}}(nz)
      end

      for i = 1:nz
         contactObj1[i] = nothing
         contactObj2[i] = nothing
      end
      new(collSuperObjs, noCPairs, AABB, dummyObject3D, length(collSuperObjs), nz, allPossibleContactPairsInz,
          z, contactPoint1, contactPoint2, contactNormal, contactObj1, contactObj2)
   end
end

initializeContactDetection!(ch::Modia3D.AbstractContactDetection, collSuperObjs::Array{Array{Modia3D.AbstractObject3Ddata}}, noCPairs::Array{Array{Int64,1}}) = error("No contact detection handler defined.")
selectContactPairs!(ch::Modia3D.AbstractContactDetection)            = error("No contact detection handler defined.")
getDistances!(ch::Modia3D.AbstractContactDetection)                  = error("No contact detection handler defined.")
setComputationFlag(ch::Modia3D.AbstractContactDetection)             = error("No contact detection handler defined.")
closeContactDetection!(ch::Modia3D.AbstractContactDetection)         = error("No contact detection handler defined.")

include(joinpath(Modia3D.path, "src", "contactDetection", "ContactDetectionMPR", "ContactDetectionMPR_handler.jl"))



#-------------------------------------- Gravity field ----------------------------------

struct NoGravityField <: Modia3D.AbstractGravityField
   gvec::SVector{3,Float64} # [m/s^2] Vector of gravity acceleration
   NoGravityField() = new(SVector{3,Float64}(0.0, 0.0, 0.0))
end
gravityAcceleration(grav::NoGravityField, r_abs::AbstractVector) = grav.gvec


"""
    UniformGravityField(;g=9.81, n=[0,1,0])

Return a UniformGravityField struct.

# Arguments
- `g::Float64`: Gravity constant
- `n::AbstractVector`: Direction of gravity

# Example
```julia
import Modia3D

grav = Modia3D.UniformGravityField()
   r = Modia3D.EarthRadius
   g = gravityAcceleration(grav,r)
```
"""
struct UniformGravityField <: Modia3D.AbstractGravityField
   gvec::SVector{3,Float64} # [m/s^2] Vector of gravity acceleration

   function UniformGravityField(;g=9.81, n=[0,-1,0])
      @assert(g >= 0.0)
      new(g*normalize(n))
   end
end
gravityAcceleration(grav::UniformGravityField, r_abs::AbstractVector) = grav.gvec


const G           = 6.67408e-11  # [m3/(kg.s2)]  Newtonian constant of gravitation (https://en.wikipedia.org/wiki/Gravitational_constant)
const EarthMass   = 5.9722e24    # [kg]          Mass of earth (https://en.wikipedia.org/wiki/Earth_mass)
const EarthRadius = 6.3781e6     # [m]           Radius of earth (https://en.wikipedia.org/wiki/Earth_radius)


"""
    PointGravityField(mass), PointGravityField(;mue=G*EarthMass)

Return a PointGravityField struct with the gravity field constant mue (mue = G*mass).


# Example
```julia
import Modia3D

grav = Modia3D.PointGravityField()   # Gravity field of earth
   r = Modia3D.EarthRadius
   g = gravityAcceleration(grav,r)
```
"""
struct PointGravityField <: Modia3D.AbstractGravityField
   mue::Float64    # [m3/s2] Gravity field constant (default = field constant of earth)

   function PointGravityField(mass)
      @assert(mass >= 0.0)
      mue = G*mass
      new(mue)
   end
   function PointGravityField(;mue = G*EarthMass)
      @assert(mue >= 0.0)
      new(mue)
   end
end
gravityAcceleration(grav::PointGravityField, r_abs::AbstractVector) = -(grav.mue/dot(r_abs,r_abs))*normalize(r_abs)



#-------------------------------------- Global Scene Options -------------------------------

struct SceneOptions
   # Handler used for contact detection
   contactDetection::Modia3D.AbstractContactDetection
   nz_max::Int                    # Maximum number of zero crossing functions used for
                                  # contact detection with variable step integrators

   # Gravity field
   gravityField::Modia3D.AbstractGravityField

   # Animation
   enableVisualization::Bool      # = true, if animation is enabled
   visualizeGravity::Bool         # = true, if gravity field shall be visualized (acceleration vector or field center)
   visualizeFrames::Bool          # = true, if all frames shall be visualized
   visualizeConvexHulls::Bool     # = true, if convex hulls (used for contact detection) shall be visualized
   visualizeContactPoints::Bool   # = true, if contact points shall be visualized
   visualizeSupportPoints::Bool   # = true, if support points shall be visualized

   # Visual defaults
   nominalLength::Float64         # [m]     Nominal length of 3D system
   defaultFrameLength::Float64    # [m]     Default for frame length if visualizeFrames = true (but not world frame)
   defaultJointLength::Float64    # [m]     Default for the fixed length of a shape representing a joint
   defaultJointWidth::Float64     # [m]     Default for the fixed width of a shape representing a joint
   defaultForceLength::Float64    # [m]     Default for the fixed length of a shape representing a force (e.g., damper)
   defaultForceWidth::Float64     # [m]     Default for the fixed width of a shape representing a force (e.g., spring, bushing)
   defaultBodyDiameter::Float64   # [m]     Default for diameter of sphere representing the center of mass of a body
   defaultWidthFraction::Float64  #         Default for shape width as a fraction of shape length
   defaultArrowDiameter::Float64  # [m]     Default for arrow diameter (e.g., of forces, torques, sensors)
   defaultN_to_m::Float64         # [N/m]   Default scaling of force arrows (length = force/defaultN_to_m)
   defaultNm_to_m::Float64        # [N.m/m] Default scaling of torque arrows (length = torque/defaultNm_to_m)

   # ContactDetection
   enableContactDetection::Bool   # = true, if contact detection is enabled

   function SceneOptions(;contactDetection       = ContactDetectionMPR_handler(),
                          nz_max                 = 100,
                          gravityField           = UniformGravityField(),
                          enableVisualization    = true,
                          visualizeGravity       = true,
                          visualizeFrames        = false,
                          visualizeConvexHulls   = true,
                          visualizeContactPoints = false,
                          visualizeSupportPoints = false,
                          nominalLength          = 1.0,
                          defaultFrameLength     = 0.2*nominalLength,
                          defaultJointLength     = nominalLength/10,
                          defaultJointWidth      = nominalLength/20,
                          defaultForceLength     = nominalLength/10,
                          defaultForceWidth      = nominalLength/20,
                          defaultBodyDiameter    = nominalLength/9,
                          defaultWidthFraction   = 20,
                          defaultArrowDiameter   = nominalLength/40,
                          defaultN_to_m          = 1000,
                          defaultNm_to_m         = 1000,
                          enableContactDetection = true)
      @assert(nz_max > 0)
      @assert(nominalLength > 0.0)
      @assert(defaultFrameLength > 0.0)
      @assert(defaultJointLength > 0.0)
      @assert(defaultJointWidth > 0.0)
      @assert(defaultForceLength > 0.0)
      @assert(defaultForceWidth > 0.0)
      @assert(defaultBodyDiameter >= 0.0)
      @assert(defaultArrowDiameter > 0.0)
      @assert(defaultN_to_m > 0.0)
      @assert(defaultNm_to_m  > 0.0)

      new(contactDetection,
          nz_max,
          gravityField,
          enableVisualization,
          visualizeGravity,
          visualizeFrames,
          visualizeConvexHulls,
          visualizeContactPoints,
          visualizeSupportPoints,
          nominalLength,
          defaultFrameLength,
          defaultJointLength,
          defaultJointWidth,
          defaultForceLength,
          defaultForceWidth,
          defaultBodyDiameter,
          defaultWidthFraction,
          defaultArrowDiameter,
          defaultN_to_m,
          defaultNm_to_m,
          enableContactDetection)
   end
end


#-------------------------------------- Scene -------------------------------
mutable struct Scene
   autoCoordsys::Graphics.CoordinateSystem   # Coordinate system that is automatically included (e.g. due to visualizeFrames=true)
   stack::Vector{Object3D}                   # Stack to traverse frames
   buffer::Vector{Object3D}

   options::SceneOptions                     # Global options defined for the scene

   visualize::Bool                           # = true, if visualization elements available
   collide::Bool                             # = true, if elements for contact detection available

   # initialization of analysis
   initAnalysis::Bool                        # = true, if analysis is initialized
   initSuperObj::Bool                        # = true, if super objects are initialized
   analysis::ModiaMath.AnalysisType          # Type of analysis
   superObjs::Array{SuperObjsRow,1}          # super objects
   treeVisu::Vector{Object3D}
   treeAccVelo::Vector{Object3D}
   tree::Vector{Object3D}                    # Spanning tree of the frames in depth-first order (without world)
   cutJoints::Vector{Modia3D.AbstractJoint}  # Vector of all cut-joints
   allVisuElements::Vector{Object3D}         # Object3Ds (including Object3Ds.data) that shall be visualized
   noCPairs::Array{Array{Int64,1}}           # Indices of frames (with respect to collSuperObjs) that can't collide in general (e.g. objects are connected via joints)
   noCPairsHelp::Dict{Modia3D.AbstractJoint,Array{Int64,1}}
   AABB::Array{Array{Basics.BoundingBox}}    # Bounding boxes of elements that can collide
   #forceElements::Array{Int64,1}

   uniqueSignals::Array{Modia3D.AbstractSignal}
   uniqueForceTorques::Array{Modia3D.AbstractForceTorque}
   potentialVarInput::Array{ModiaMath.RealScalar}
   potentialVarOutput::Array{ModiaMath.RealScalar}
   flowVarInput::Array{ModiaMath.RealScalar}
   flowVarOutput::Array{ModiaMath.RealScalar}
   Scene(sceneOptions::SceneOptions = SceneOptions()) =
              new(Graphics.CoordinateSystem(sceneOptions.defaultFrameLength),
                  Vector{Object3D}[],
                  Vector{Object3D}[],
                  sceneOptions,
                  false,
                  false,
                  false,
                  false,
                  ModiaMath.KinematicAnalysis,
                  Array{SuperObjsRow,1}(),
                  Vector{Object3D}[],
                  Vector{Object3D}[],
                  Vector{Object3D}[],
                  Vector{Modia3D.AbstractJoint}[],
                  Vector{Object3D}[],
                  Array{Array{Int64,1},1}(),
                  Dict{Modia3D.AbstractJoint,Array{Int64,1}}(),
                  Array{Array{Basics.BoundingBox,1},1}(),
                  Array{Modia3D.AbstractSignal,1}(),
                  Array{Modia3D.AbstractForceTorque,1}(),
                  Array{ModiaMath.RealScalar,1}(),
                  Array{ModiaMath.RealScalar,1}(),
                  Array{ModiaMath.RealScalar,1}(),
                  Array{ModiaMath.RealScalar,1}())
end
