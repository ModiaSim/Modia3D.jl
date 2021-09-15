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

- `DummyObject3D::Modia3D.AbstractObject3DFeature`: A dummy Object3D that can be used in the struct as element of a vector of Object3Ds
  to fill the vector with a dummy value of the correct type.
"""
mutable struct ContactPairs
    # Solid shapes used in contact detection (provided by Object3D handler)
    allowedToMove::Vector{Union{Bool,Nothing}}
    dummyObject3D::Modia3D.AbstractObject3DFeature     # Dummy Object3D for non-used elements of z-Vector.

    # Dimensions
    ne::Int                                         # length(collSuperObjs)
    nz::Int                                         # length(z)
    nzContact::Int                                  # length(z | z has contact) length of z where zi has contact

    function ContactPairs(world::Composition.Object3D, AABB::Vector{Vector{Basics.BoundingBox}}, superObjs::Vector{SuperObjsRow},
                        allowedToMove::Vector{Union{Bool,Nothing}}, visualizeBoundingBox::Bool, nVisualContSupPoints::Int,
                        visualizeContactPoints::Bool, visualizeSupportPoints::Bool, defaultContactSphereDiameter::Float64)
        @assert(length(superObjs) > 0)
        @assert(nVisualContSupPoints > 0)
        dummyObject3D = Composition.emptyObject3DFeature

        lengthCollSuperObjs = length(superObjs)
        if lengthCollSuperObjs <= 1
            error("There's nothing to collide. All Object3Ds are rigidly connected.")
        end

        nzContact = 0
        nz = 2

        if lengthCollSuperObjs > 2
            nz = lengthCollSuperObjs
        end

        amountVisuPoints = 1
        amountSuppPoints = 1
        if visualizeContactPoints
            addContactVisuObjToWorld!(world, nVisualContSupPoints, defaultContactSphereDiameter)
        end
        if visualizeSupportPoints
            addSupportVisuObjToWorld!(world, nVisualContSupPoints, defaultContactSphereDiameter)
        end

        if visualizeBoundingBox
            addAABBVisuToWorld!(world, AABB)
        end


        new(allowedToMove, dummyObject3D, lengthCollSuperObjs, nz, nzContact)
    end
end

initializeContactDetection!(ch::Modia3D.AbstractContactDetection, collSuperObjs::Vector{Vector{Modia3D.AbstractObject3DFeature}}, noCPairs::Vector{Vector{Int64}}) = error("No contact detection handler defined.")
selectContactPairsWithEvent!(ch::Modia3D.AbstractContactDetection)   = error("No contact detection handler defined.")
selectContactPairsNoEvent!(ch::Modia3D.AbstractContactDetection)     = error("No contact detection handler defined.")
getDistances!(ch::Modia3D.AbstractContactDetection)                  = error("No contact detection handler defined.")
setComputationFlag(ch::Modia3D.AbstractContactDetection)             = error("No contact detection handler defined.")
closeContactDetection!(ch::Modia3D.AbstractContactDetection)         = error("No contact detection handler defined.")

include(joinpath(Modia3D.path, "src", "contactDetection", "ContactDetectionMPR", "ContactDetectionMPR_handler.jl"))



#-------------------------------------- Gravity field ----------------------------------

"""
    gravityField = NoGravityField()

Generate an instance of type `NoGravityField` that defines no gravity.
"""
struct NoGravityField <: Modia3D.AbstractGravityField
   gvec::SVector{3,Float64} # [m/s^2] Vector of gravity acceleration
   NoGravityField() = new(SVector{3,Float64}(0.0, 0.0, 0.0))
end
gravityAcceleration(grav::NoGravityField, r_abs::SVector{3,Float64})::SVector{3,Float64} = grav.gvec


"""
    UniformGravityField(;g=9.81, n=[0,-1,0])

Generate an instance of type `UniformGravityField` that defines
a uniform gravity field with gravitational acceleration `g` in
direction `n`.

# Arguments
- `g::Float64`: Gravitational acceleration
- `n::AbstractVector`: Direction of gravitational acceleration

# Example
```julia
import Modia3D

grav = Modia3D.UniformGravityField()
   r = Modia3D.EarthRadius
   g = gravityAcceleration(grav,r)  # g is independent of r
```
"""
struct UniformGravityField <: Modia3D.AbstractGravityField
   gvec::SVector{3,Float64} # [m/s^2] Vector of gravity acceleration

   function UniformGravityField(;g=9.81, n=[0,-1,0])
      @assert(g >= 0.0)
      new(g*normalize(n))
   end
end
gravityAcceleration(grav::UniformGravityField, r_abs::SVector{3,Float64})::SVector{3,Float64} = grav.gvec


const G           = 6.67408e-11  # [m3/(kg.s2)]  Newtonian constant of gravitation (https://en.wikipedia.org/wiki/Gravitational_constant)
const EarthMass   = 5.9722e24    # [kg]          Mass of earth (https://en.wikipedia.org/wiki/Earth_mass)
const EarthRadius = 6.3781e6     # [m]           Radius of earth (https://en.wikipedia.org/wiki/Earth_radius)


"""
    PointGravityField([mass|;mue=G*EarthMass])

Generate an instance of type `PointGravityField` that defines
a point gravity field of `mass` or gravity field constant `mue`.
The gravity center is located at the world origin, i.e. the
gravitational acceleration decreases quadratically with distance
from world.

# Example
```julia
import Modia3D

grav = Modia3D.PointGravityField()   # Gravity field of earth
   r = Modia3D.EarthRadius
   g = gravityAcceleration(grav,r)  # g is singular at r=0
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
gravityAcceleration(grav::PointGravityField, r_abs::SVector{3,Float64})::SVector{3,Float64} = -(grav.mue/dot(r_abs,r_abs))*normalize(r_abs)


"""
    sceneUpDir = upwardsDirection(gravityField::Modia3D.AbstractGravityField)

Determine upwards direction of scene depending on main gravity field direction.

sceneUpDir: -/+1 = -/+x-direction; -/+2 = -/+y-direction; -/+3 = -/+z-direction
"""
function upwardsDirection(gravityField::Modia3D.AbstractGravityField)
    sceneUpDir = +2
    if typeof(gravityField) == UniformGravityField
        gv = gravityField.gvec
        if norm(gv) > 1e-8
            if (abs(gv[1]) > abs(gv[2])) && (abs(gv[1]) > abs(gv[3]))
                if gv[1] < 0.0
                    sceneUpDir = +1
                else
                    sceneUpDir = -1
                end
            elseif (abs(gv[2]) > abs(gv[3])) && (abs(gv[2]) > abs(gv[1]))
                if gv[2] < 0.0
                    sceneUpDir = +2
                else
                    sceneUpDir = -2
                end
            else
                if gv[3] < 0.0
                    if gv[3] < 0.0
                        sceneUpDir = +3
                    else
                        sceneUpDir = -3
                    end
                end
            end
        end
    end
    return sceneUpDir
end

function camRotation(beta, alpha)
    # beta = longitude angle about y-axis of view ref frame
    # alpha = latitude angle about -x-axis of beta-rotated frame
    # R_cam = transformation matrix v_cam = R_cam * v_view
    cbe = cos(beta)
    sbe = sin(beta)
    cal = cos(alpha)
    sal = sin(alpha)
    R_cam = [cbe  0.0  -sbe; -sal*sbe  cal  -sal*cbe; cal*sbe  sal  cal*cbe]
end

"""
    (r_Camera, R_Camera) = cameraPosition(distance, longitude, latitude, cameraUpDir, sceneUpDir)

Determine camera/light position and orientation.

distance:    Distance between world frame and camera position.
longitude:   Longitude angle of camera position (0 = -y/-z/-x direction).
latitude:    Latitude angle of camera position (0 = horizontal).
cameraUpDir: Upwards direction of camera.
sceneUpDir:  Upwards direction of scene.

r_Camera: Absolute position vector of camera/light.
R_Camera: Absolute rotation matrix of camera/light.
"""
function cameraPosition(distance, longitude, latitude, cameraUpDir, sceneUpDir)
    R_Camera = camRotation(longitude, latitude)
    r_Camera = R_Camera' * [0.0, 0.0, 1.0] * distance
    if sceneUpDir == -1
        R_view = [0 1 0; -1 0 0; 0 0 1]  # scene upwards := -x direction
    elseif sceneUpDir == 1
        R_view = [0 0 1; +1 0 0; 0 1 0]  # scene upwards := +x direction
    elseif sceneUpDir == -2
        R_view = [0 0 1; 0 -1 0; 1 0 0]  # scene upwards := -y direction
    elseif sceneUpDir == -3
        R_view = [1 0 0; 0 0 -1; 0 1 0]  # scene upwards := -z direction
    elseif sceneUpDir == +3
        R_view = [0 1 0; 0 0 +1; 1 0 0]  # scene upwards := +z direction
    else
        R_view = [1 0 0; 0 +1 0; 0 0 1]  # scene upwards := +y direction
    end
    if cameraUpDir == 1
        R_view = [0 1 0; 0 0 1; 1 0 0] * R_view  # camera upwards := +x direction
    elseif cameraUpDir == 3
        R_view = [0 0 1; 1 0 0; 0 1 0] * R_view  # camera upwards := +z direction
    else
        # camera upwards := +y direction
    end
    r_Camera = SVector{3,Float64}(R_view' * r_Camera)
    R_Camera = SMatrix{3,3,Float64}(R_Camera * R_view)
    return (r_Camera, R_Camera)
end



#-------------------------------------- Global Scene Options -------------------------------
struct SceneOptions <: Modia3D.AbstractSceneOptions

    # Gravity field
    gravityField::Modia3D.AbstractGravityField

    # Multibody structure
    useOptimizedStructure::Bool    # = true, if the optimized structure (with super objects, and common inertia) is used

    # Contact detection
    contactDetection::Modia3D.AbstractContactDetection
    nVisualContSupPoints::Int               # amount of visual contact or support points
    gap::Float64
    enableContactDetection::Bool            # = true, if contact detection is enabled
    defaultContactSphereDiameter::Float64   # = true, if contact points are visualized
    elasticContactReductionFactor::Float64  # c_res_used = c_res * elasticContactReductionFactor (> 0)

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

    # Animation
    enableVisualization::Bool             # = true, if online animation is enabled
    animationFile::Union{Nothing,String}  # path&name of animation file
    visualizeGravity::Bool                # = true, if gravity field shall be visualized (acceleration vector or field center)
    visualizeFrames::Bool                 # = true, if all frames shall be visualized
    visualizeConvexHulls::Bool            # = true, if convex hulls (used for contact detection) shall be visualized
    visualizeBoundingBox::Bool            # = true, if AABB's are visualized
    visualizeContactPoints::Bool          # = true, if contact points shall be visualized
    visualizeSupportPoints::Bool          # = true, if support points shall be visualized
    cameraDistance::Float64               # Distance between world frame and camera position
    cameraLongitude::Float64              # Longitude angle of camera position (0 = -y/-z/-x direction)
    cameraLatitude::Float64               # Latitude angle of camera position (0 = horizontal)
    lightDistance::Float64                # Distance between world frame and light position
    lightLongitude::Float64               # Longitude angle of light position (0 = -y/-z/-x direction)
    lightLatitude::Float64                # Latitude angle of light position (0 = horizontal)

    function SceneOptions(;gravityField                  = UniformGravityField(),
                           useOptimizedStructure         = true,
                           contactDetection              = ContactDetectionMPR_handler(),
                           nVisualContSupPoints          = 5,
                           gap                           = 0.001,
                           enableContactDetection        = true,
                           defaultContactSphereDiameter  = 0.1,
                           elasticContactReductionFactor = 1.0,
                           nominalLength                 = 1.0,
                           defaultFrameLength            = 0.2*nominalLength,
                           defaultJointLength            = nominalLength/10,
                           defaultJointWidth             = nominalLength/20,
                           defaultForceLength            = nominalLength/10,
                           defaultForceWidth             = nominalLength/20,
                           defaultBodyDiameter           = nominalLength/9,
                           defaultWidthFraction          = 20,
                           defaultArrowDiameter          = nominalLength/40,
                           defaultN_to_m                 = 1000,
                           defaultNm_to_m                = 1000,
                           enableVisualization           = true,
                           animationFile                 = nothing,
                           visualizeGravity              = true,
                           visualizeFrames               = false,
                           visualizeConvexHulls          = true,
                           visualizeBoundingBox          = false,
                           visualizeContactPoints        = false,
                           visualizeSupportPoints        = false,
                           cameraDistance                = 10.0*nominalLength,
                           cameraLongitude               = 30/180*pi,
                           cameraLatitude                = 15/180*pi,
                           lightDistance                 = 10.0*nominalLength,
                           lightLongitude                = 60/180*pi,
                           lightLatitude                 = 45/180*pi)
        @assert(nVisualContSupPoints > 0)
        @assert(gap > 0.0)
        @assert(defaultContactSphereDiameter > 0.0)
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
        @assert(cameraDistance > 0.0)
        @assert(lightDistance > 0.0)

        sceneOptions = new(gravityField,
            useOptimizedStructure,
            contactDetection,
            nVisualContSupPoints,
            gap,
            enableContactDetection,
            defaultContactSphereDiameter,
            elasticContactReductionFactor,
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
            enableVisualization,
            animationFile,
            visualizeGravity,
            visualizeFrames,
            visualizeConvexHulls,
            visualizeBoundingBox,
            visualizeContactPoints,
            visualizeSupportPoints,
            cameraDistance,
            cameraLongitude,
            cameraLatitude,
            lightDistance,
            lightLongitude,
            lightLatitude)

        sceneOptions.contactDetection.visualizeContactPoints       = visualizeContactPoints
        sceneOptions.contactDetection.visualizeSupportPoints       = visualizeSupportPoints
        sceneOptions.contactDetection.defaultContactSphereDiameter = defaultContactSphereDiameter
        return sceneOptions
    end
end

struct animationData
    position::SVector{3,Float64}    # abs. Object3D position
    quaternion::SVector{4,Float64}  # abs. Object3D quaternion
end

struct animationStep
    time::Float64                      # simulation time
    objectData::Vector{animationData}  # animation data of Object3Ds
end


#-------------------------------------- Scene -------------------------------
"""
    Scene(; kwargs...)

Defines global properties of the system, such as the gravity field. Exactly one [Object3D](@ref) must have a `Scene` feature defined. This Object3D is used as inertial system (world, root) and is not allowed to have a parent Object3D.

| Keyword arguments             | defaults                                |
|:------------------------------|:----------------------------------------|
| `gravityField`                  | [`UniformGravityField`](@ref)`()`         |
| `enableVisualization`           | true                                    |
| `visualizeFrames`               | false                                   |
| `nominalLength`                 | 1.0                                     |
| `defaultFrameLength`            | 0.2*`nominalLength`                     |
| `useOptimizedStructure`         | true                                    |
| `enableContactDetection`        | true                                    |
| `contactDetection`              | [`ContactDetectionMPR_handler`](@ref)`()`|
| `elasticContactReductionFactor` | 1.0                                     |
| `visualizeBoundingBox`          | false                                   |
| `visualizeContactPoints`        | false                                   |
| `visualizeSupportPoints`        | false                                   |
| `nVisualContSupPoints`          | 5                                       |
| `defaultContactSphereDiameter`  | 0.1                                     |
| `animationFile`                 | nothing                                 |
| `cameraDistance`                | 10.0*`nominalLength`                      |
| `cameraLongitude`               | 30/180*pi                               |
| `cameraLatitude`                | 15/180*pi                               |
| `lightDistance`                 | 10.0*`nominalLength`                      |
| `lightLongitude`                | 60/180*pi                               |
| `lightLatitude`                 | 45/180*pi                               |


# Arguments
- `gravityField::Modia3D.AbstractGravityField`: Gravity field of the scene. Supported values:
   - [`NoGravityField`](@ref),
   - [`UniformGravityField`](@ref),
   - [`PointGravityField`](@ref)

- `enableVisualization::Bool`: = true, to enable online animation with DLR SimVis. If SimVis is not installed, this flag has no effect.

- `visualizeFrames::Bool`: = true, to visualize the coordinate system of every [Object3D](@ref) that is not explicitly switched off.

- `nominalLength::Float64`: Nominal length in [m].

- `defaultFrameLength::Float64`: Default frame length in [m] for visualizing little [CoordinateSystem](@ref).

- `useOptimizedStructure::Bool`: = true, if pre-processing the whole system. For example, computing the common mass, common center of mass and common inertia tensor of all rigidly connected Object3Ds that have mass properties.

- `enableContactDetection::Bool`: = true, if contact detection is enable, see [Collision Handling](@ref).

- `contactDetection::Modia3D.AbstractContactDetection`: Handler used for contact detection e.g., to determine the smallest distance between two objects.

- `elasticContactReductionFactor::Float64`: (> 0.0)
  - ``usedContactCompliance = contactCompliance * elasticContactReductionFactor``

- `visualizeBoundingBox::Bool`: Flag enabled for visualizing Axis Aligned Bounding Box (AABB) for all solid shapes allowed to collide

- `visualizeContactPoints::Bool`: Flag enabled for visualizing contact points, and `enableVisualization = true`

- `visualizeSupportPoints::Bool`: Flag enabled for visualizing support points, and `enableVisualization = true`

- `nVisualContSupPoints::Int64`: defines how many contact as well as support points are visualized (there is a warning if you'll need to increase it)

- `defaultContactSphereDiameter::Float64`: diameter of [Sphere](@ref) in [m] used for contact and support point visualization

- `animationFile::String`: only if a valid path and name of the animation file is set (it must be a .json file) a json file is exported

- `cameraDistance::Float64`: distance between world frame and camera position

- `cameraLongitude::Float64`: longitude angle of camera position (0 = -y/-z/-x direction)

- `cameraLatitude::Float64`: latitude angle of camera position (0 = horizontal)

- `lightDistance::Float64`: distance between world frame and light position

- `lightLongitude::Float64`: longitude angle of light position (0 = -y/-z/-x direction)

- `lightLatitude::Float64`: latitude angle of light position (0 = horizontal)
"""
mutable struct Scene
    name::String                              # model name
    autoCoordsys::Shapes.CoordinateSystem   # Coordinate system that is automatically included (e.g. due to visualizeFrames=true)
    stack::Vector{Object3D}                   # Stack to traverse objs
    buffer::Vector{Object3D}                  # stores all roots of a super obj

    options::SceneOptions                     # Global options defined for the scene

    visualize::Bool                           # = true, if visualization elements available
    collide::Bool                             # = true, if elements for contact detection available

    # initialization of analysis
    initAnalysis::Bool                        # = true, if analysis is initialized
    initSuperObj::Bool                        # = true, if super objects are initialized
    initMassComp::Bool                        # = true, if mass computation for optimized structure is initialized
    analysis::Modia3D.AnalysisType            # Type of analysis
    superObjs::Vector{SuperObjsRow}          # super objects

    treeAccVelo::Vector{Object3D}
    tree::Vector{Object3D}                    # Spanning tree of the frames in depth-first order (without world)
    treeForComputation::Vector{Object3D}      # Tree that is used for the computation
                                              # treeForComputation = options.useOptimizedStructure ? treeAccVelo : tree

    #cutJoints::Vector{Modia3D.AbstractJoint}  # Vector of all cut-joints
    allVisuElements::Vector{Object3D}         # all Object3Ds that should be visualized (for communiating with SimVis)
    updateVisuElements::Vector{Object3D}      # all Object3Ds that are visibly only e.g. visualization frames (must be updated first)
    allCollisionElements::Vector{Object3D}    # all Object3Ds, which are allowed to collide (no order, no super objects)
    noCPairs::Vector{Vector{Int64}}           # Indices of frames (with respect to collSuperObjs) that can't collide in general (e.g. objects are connected via joints)
    noCPairsHelp::Dict{Modia3D.AbstractJoint,Vector{Int64}}
    allowedToMove::Vector{Union{Bool,Nothing}}
    AABB::Vector{Vector{Basics.BoundingBox}}  # Bounding boxes of elements that can collide
    zStartIndex::Int                          # start index of collision zero crossing functions
    #forceElements::Vector{Int64}
    exportAnimation::Bool                     # animation file export is enabled
    animation::Vector{animationStep}          # animation data of visible Object3Ds
    outputCounter::Int64                      # animation/visualization output step counter

    # Data specific to a particular joint type
    revolute::Vector{Revolute}
    prismatic::Vector{Prismatic}
    freeMotion::Vector{FreeMotion}

    function Scene(sceneOptions::SceneOptions = SceneOptions())
        exportAnimation = false
        if !isnothing(sceneOptions.animationFile)
            (base, ext) = splitext(sceneOptions.animationFile)
            if ext == ".json"
                exportAnimation = true
            else
                @warn("Extension of animationFile=$(sceneOptions.animationFile) is not 'json'.\n-> Animation export is disabled.")
            end
        end
        new("Scene",
            Shapes.CoordinateSystem(length=sceneOptions.defaultFrameLength),
            Vector{Object3D}[],
            Vector{Object3D}[],
            sceneOptions,
            false,
            false,
            false,
            false,
            false,
            Modia3D.KinematicAnalysis,
            Vector{SuperObjsRow}[],
            Vector{Object3D}[],
            Vector{Object3D}[],
            Vector{Object3D}[],
            Vector{Object3D}[],
            Vector{Object3D}[],
            Vector{Object3D}[],
            Vector{Vector{Int64}}[],
            Dict{Modia3D.AbstractJoint,Vector{Int64}}(),
            Vector{Union{Bool}}[],
            Vector{Vector{Basics.BoundingBox}}[],
            1,
            exportAnimation,
            Vector{animationStep}[],
            0,
            Revolute[],
            Prismatic[],
            FreeMotion[])
    end
end
