# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#



#-------------------------------------- Default Renderer -------------------------------

initializeVisualization(renderer::Modia3D.AbstractRenderer, allVisuElements::Vector{Object3D{F}}) where F <: AbstractFloat = error("No renderer defined.")
visualize!(renderer::Modia3D.AbstractRenderer, time) = error("No renderer defined.")
closeVisualization(renderer::Modia3D.AbstractRenderer)        = error("No renderer defined.")



#-------------------------------------- Gravity field ----------------------------------

"""
    gravityField = NoGravityField()

Generate an instance of type `NoGravityField` that defines no gravity.
"""
struct NoGravityField <: Modia3D.AbstractGravityField
   gvec::SVector{3,Float64} # [m/s^2] Vector of gravity acceleration
   NoGravityField() = new(SVector{3,Float64}(0.0, 0.0, 0.0))
end
gravityAcceleration(grav::NoGravityField, r_abs::SVector{3,F}) where F <: AbstractFloat = SVector{3,F}(grav.gvec)


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
gravityAcceleration(grav::UniformGravityField, r_abs::SVector{3,F}) where F <: AbstractFloat = SVector{3,F}(grav.gvec)


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
gravityAcceleration(grav::PointGravityField, r_abs::SVector{3,F}) where F <: AbstractFloat = SVector{3,F}(-(grav.mue/dot(r_abs,r_abs))*normalize(r_abs))


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



#-------------------------------------- Global SceneOptions -------------------------------
struct SceneOptions{F <: AbstractFloat}
    # Gravity field
    gravityField::Modia3D.AbstractGravityField

    # Multibody structure
    useOptimizedStructure::Bool    # = true, if the optimized structure (with super objects, and common inertia) is used

    ### Contact detection ###
    enableContactDetection::Bool            # = true, if contact detection is enabled
    contactDetection::Modia3D.AbstractContactDetection
    elasticContactReductionFactor::F  # c_res_used = c_res * elasticContactReductionFactor (> 0)
    gap::Float64


    ### Animation and Visualization ###
    enableVisualization::Bool             # = true, if online animation is enabled
    animationFile::Union{Nothing,String}  # path&name of animation file
    visualizeFrames::Bool                 # = true, if all frames shall be visualized
    visualizeBoundingBox::Bool            # = true, if AABB's are visualized
    visualizeContactPoints::Bool          # = true, if contact points shall be visualized
    visualizeSupportPoints::Bool          # = true, if support points shall be visualized
    # Visual and Animation defaults
    nominalLength::Float64                # [m] Nominal length of 3D system
    defaultFrameLength::Float64           # [m] Default for frame length if visualizeFrames = true (but not world frame)
    nVisualContSupPoints::Int             # amount of visual contact or support points
    defaultContactSphereDiameter::Float64 # = true, if contact points are visualized
    cameraDistance::Float64               # Distance between world frame and camera position
    cameraLongitude::Float64              # Longitude angle of camera position (0 = -y/-z/-x direction)
    cameraLatitude::Float64               # Latitude angle of camera position (0 = horizontal)
    lightDistance::Float64                # Distance between world frame and light position
    lightLongitude::Float64               # Longitude angle of light position (0 = -y/-z/-x direction)
    lightLatitude::Float64                # Latitude angle of light position (0 = horizontal)

    function SceneOptions{F}(;gravityField    = UniformGravityField(),
            useOptimizedStructure         = true,
            enableContactDetection        = true,
            contactDetection              = ContactDetectionMPR_handler(),
            elasticContactReductionFactor = F(1.0),
            gap                           = 0.001,
            enableVisualization           = true,
            animationFile                 = nothing,
            visualizeFrames               = false,
            visualizeBoundingBox          = false,
            visualizeContactPoints        = false,
            visualizeSupportPoints        = false,
            nominalLength                 = 1.0,
            defaultFrameLength            = 0.2*nominalLength,
            nVisualContSupPoints          = 5,
            defaultContactSphereDiameter  = 0.1,
            cameraDistance                = 10.0*nominalLength,
            cameraLongitude               = 30/180*pi,
            cameraLatitude                = 15/180*pi,
            lightDistance                 = 10.0*nominalLength,
            lightLongitude                = 60/180*pi,
            lightLatitude                 = 45/180*pi) where F <: AbstractFloat
        @assert(gap > 0.0)
        @assert(nominalLength > 0.0)
        @assert(defaultFrameLength > 0.0)
        @assert(nVisualContSupPoints > 0)
        @assert(defaultContactSphereDiameter > 0.0)
        @assert(cameraDistance > 0.0)
        @assert(lightDistance > 0.0)

        sceneOptions = new(gravityField,
            useOptimizedStructure,
            enableContactDetection,
            contactDetection,
            elasticContactReductionFactor,
            gap,
            enableVisualization,
            animationFile,
            visualizeFrames,
            visualizeBoundingBox,
            visualizeContactPoints,
            visualizeSupportPoints,
            nominalLength,
            defaultFrameLength,
            nVisualContSupPoints,
            defaultContactSphereDiameter,
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

| Keyword arguments               | defaults                                |
|:--------------------------------|:----------------------------------------|
| `gravityField`                  | [`UniformGravityField`](@ref)`()`       |
| `useOptimizedStructure`         | true                                    |
| `enableContactDetection`        | true                                    |
| `mprTolerance`                   | 1.0e-20                                 |
| `elasticContactReductionFactor` | 1.0                                     |
| `enableVisualization`           | true                                    |
| `animationFile`                 | nothing                                 |
| `visualizeFrames`               | false                                   |
| `visualizeBoundingBox`          | false                                   |
| `visualizeContactPoints`        | false                                   |
| `visualizeSupportPoints`        | false                                   |
| `nominalLength`                 | 1.0                                     |
| `defaultFrameLength`            | 0.2*`nominalLength`                     |
| `nVisualContSupPoints`          | 5                                       |
| `defaultContactSphereDiameter`  | 0.1                                     |
| `cameraDistance`                | 10.0*`nominalLength`                    |
| `cameraLongitude`               | 30/180*pi                               |
| `cameraLatitude`                | 15/180*pi                               |
| `lightDistance`                 | 10.0*`nominalLength`                    |
| `lightLongitude`                | 60/180*pi                               |
| `lightLatitude`                 | 45/180*pi                               |


# Arguments
- `gravityField::Modia3D.AbstractGravityField`: Gravity field of the scene. Supported values:
   - [`NoGravityField`](@ref),
   - [`UniformGravityField`](@ref),
   - [`PointGravityField`](@ref)

- `useOptimizedStructure::Bool`: = true, if pre-processing the whole system. For example, computing the common mass, common center of mass and common inertia tensor of all rigidly connected Object3Ds that have mass properties.

- `enableContactDetection::Bool`: = true, if contact detection is enable, see [Collision Handling](@ref).

- `mprTolerance::1.0e-20`: Local tolerance used for terminating the mpr algorithm. Changing this value might improve speed.

- `elasticContactReductionFactor::Float64`: (> 0.0)
  - ``usedContactCompliance = contactCompliance * elasticContactReductionFactor``

- `enableVisualization::Bool`: = true, to enable online animation with DLR SimVis. If SimVis is not installed, this flag has no effect.

- `animationFile::String`: only if a valid path and name of the animation file is set (it must be a .json file) a json file is exported

- `visualizeFrames::Bool`: = true, to visualize the coordinate system of every [Object3D](@ref) that is not explicitly switched off.

- `visualizeBoundingBox::Bool`: Flag enabled for visualizing Axis Aligned Bounding Box (AABB) for all solid shapes allowed to collide

- `visualizeContactPoints::Bool`: Flag enabled for visualizing contact points, and `enableVisualization = true`

- `visualizeSupportPoints::Bool`: Flag enabled for visualizing support points, and `enableVisualization = true`

- `nominalLength::Float64`: Nominal length in [m].

- `defaultFrameLength::Float64`: Default frame length in [m] for visualizing little [CoordinateSystem](@ref).

- `nVisualContSupPoints::Int64`: defines how many contact as well as support points are visualized (there is a warning if you'll need to increase it)

- `defaultContactSphereDiameter::Float64`: diameter of [Sphere](@ref) in [m] used for contact and support point visualization

- `cameraDistance::Float64`: distance between world frame and camera position

- `cameraLongitude::Float64`: longitude angle of camera position (0 = -y/-z/-x direction)

- `cameraLatitude::Float64`: latitude angle of camera position (0 = horizontal)

- `lightDistance::Float64`: distance between world frame and light position

- `lightLongitude::Float64`: longitude angle of light position (0 = -y/-z/-x direction)

- `lightLatitude::Float64`: latitude angle of light position (0 = horizontal)
"""
mutable struct Scene{F <: AbstractFloat} <: Modia3D.AbstractScene
    name::String                              # model name
    autoCoordsys::Shapes.CoordinateSystem     # Coordinate system that is automatically included (e.g. due to visualizeFrames=true)
    stack::Vector{Object3D{F}}                   # Stack to traverse objs
    buffer::Vector{Object3D{F}}                  # stores all roots of a super obj

    options::SceneOptions                     # Global options defined for the scene

    visualize::Bool                           # = true, if visualization elements available
    collide::Bool                             # = true, if elements for contact detection available

    # initialization of analysis
    initAnalysis::Bool                        # = true, if analysis is initialized
    initSuperObj::Bool                        # = true, if super objects are initialized
    initMassComp::Bool                        # = true, if mass computation for optimized structure is initialized
    analysis::Modia3D.AnalysisType            # Type of analysis
    superObjs::Vector{SuperObjsRow}           # super objects

    treeAccVelo::Vector{Object3D{F}}
    tree::Vector{Object3D{F}}                    # Spanning tree of the frames in depth-first order (without world)
    treeForComputation::Vector{Object3D{F}}      # Tree that is used for the computation
                                              # treeForComputation = options.useOptimizedStructure ? treeAccVelo : tree

    #cutJoints::Vector{Modia3D.AbstractJoint}  # Vector of all cut-joints
    allVisuElements::Vector{Object3D{F}}         # all Object3Ds that should be visualized (for communiating with SimVis)
    updateVisuElements::Vector{Object3D{F}}      # all Object3Ds that are visibly only e.g. visualization frames (must be updated first)
    allCollisionElements::Vector{Object3D{F}}    # all Object3Ds, which are allowed to collide (no order, no super objects)
    noCPairs::Vector{Vector{Int64}}           # Indices of frames (with respect to collSuperObjs) that can't collide in general (e.g. objects are connected via joints)
    noCPairsHelp::Dict{Modia3D.AbstractJoint,Vector{Int64}}
    allowedToMove::Vector{Union{Bool,Nothing}}
    AABB::Vector{Vector{Basics.BoundingBox{F}}}  # Bounding boxes of elements that can collide
    zStartIndex::Int                          # start index of collision zero crossing functions
    forceElements::Vector{Modia3D.AbstractForceElement}
    exportAnimation::Bool                     # animation file export is enabled
    animation::Vector{animationStep}          # animation data of visible Object3Ds
    outputCounter::Int64                      # animation/visualization output step counter

    # Data specific to a particular joint type
    revolute::Vector{Revolute{F}}
    prismatic::Vector{Prismatic{F}}
    freeMotion::Vector{FreeMotion{F}}


    function Scene{F}(;gravityField          = UniformGravityField(),
            useOptimizedStructure         = true,
            enableContactDetection        = true,
            mprTolerance                  = 1.0e-20,
            elasticContactReductionFactor = F(1.0),
            gap                           = 0.001,
            enableVisualization           = true,
            animationFile                 = nothing,
            visualizeFrames               = false,
            visualizeBoundingBox          = false,
            visualizeContactPoints        = false,
            visualizeSupportPoints        = false,
            nominalLength                 = 1.0,
            defaultFrameLength            = 0.2*nominalLength,
            nVisualContSupPoints          = 5,
            defaultContactSphereDiameter  = 0.1,
            cameraDistance                = 10.0*nominalLength,
            cameraLongitude               = 30/180*pi,
            cameraLatitude                = 15/180*pi,
            lightDistance                 = 10.0*nominalLength,
            lightLongitude                = 60/180*pi,
            lightLatitude                 = 45/180*pi) where F <: AbstractFloat

        sceneOptions = SceneOptions{F}(gravityField = gravityField,
            useOptimizedStructure         = useOptimizedStructure,
            contactDetection              = ContactDetectionMPR_handler{Modia3D.MPRFloatType, F}(tol_rel = mprTolerance),
            nVisualContSupPoints          = nVisualContSupPoints,
            gap                           = gap,
            enableContactDetection        = enableContactDetection,
            defaultContactSphereDiameter  = defaultContactSphereDiameter,
            elasticContactReductionFactor = elasticContactReductionFactor,
            nominalLength                 = nominalLength,
            defaultFrameLength            = defaultFrameLength,
            enableVisualization           = enableVisualization,
            animationFile                 = animationFile,
            visualizeFrames               = visualizeFrames,
            visualizeBoundingBox          = visualizeBoundingBox,
            visualizeContactPoints        = visualizeContactPoints,
            visualizeSupportPoints        = visualizeSupportPoints,
            cameraDistance                = cameraDistance,
            cameraLongitude               = cameraLongitude,
            cameraLatitude                = cameraLatitude,
            lightDistance                 = lightDistance,
            lightLongitude                = lightLongitude,
            lightLatitude                 = lightLatitude)

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
            Vector{Object3D{F}}[],
            Vector{Object3D{F}}[],
            sceneOptions,
            false,
            false,
            false,
            false,
            false,
            Modia3D.KinematicAnalysis,
            Vector{SuperObjsRow}[],
            Vector{Object3D{F}}[],
            Vector{Object3D{F}}[],
            Vector{Object3D{F}}[],
            Vector{Object3D{F}}[],
            Vector{Object3D{F}}[],
            Vector{Object3D{F}}[],
            Vector{Vector{Int64}}[],
            Dict{Modia3D.AbstractJoint,Vector{Int64}}(),
            Vector{Union{Bool}}[],
            Vector{Vector{Basics.BoundingBox{F}}}[],
            1,
            Vector{Modia3D.AbstractForceElement}[],
            exportAnimation,
            Vector{animationStep}[],
            0,
            Revolute{F}[],
            Prismatic{F}[],
            FreeMotion{F}[])
    end
end

Scene(; kwargs...) = Scene{Float64}(; kwargs...)
