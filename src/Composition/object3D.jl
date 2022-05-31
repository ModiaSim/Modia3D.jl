# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


# Dummy feature
struct EmptyObject3DFeature <: Modia3D.AbstractObject3DFeature
end
const emptyObject3DFeature = EmptyObject3DFeature()


struct EmptyTwoObject3DObject <: Modia3D.AbstractTwoObject3DObject
end
const emptyTwoObject3DObject = EmptyTwoObject3DObject()


mutable struct InteractionManner
    gripper::Bool
    movable::Bool
    lockable::Bool
    movablePos::Int64
    originPos::Int64
    actualPos::Int64
    InteractionManner() = new(false, false, false, 0, 0, 0)

    function InteractionManner(interactionBehavior::InteractionBehavior)
        new(false, false, false, 0, 0, 0)
    end
end



"""
    Object3D(;
        parent          = nothing,
        fixedToParent   = true,
        translation     = [0.0, 0.0, 0.0],
        rotation        = [0.0, 0.0, 0.0],
        velocity        = [0.0, 0.0, 0.0],
        angularVelocity = [0.0, 0.0, 0.0],
        feature         = nothing)

Generate a new Object3D object, that is a coordinate system with associated feature that is described relatively to its (optional) parent Object3D.

Vectors `translation`, `rotation`, `velocity`, `angularVelocity` can be defined with units from package [Unitful](https://github.com/PainterQubits/Unitful.jl). If no units are provided, SI units are assumed (internally, all computations are performed with SI units, that is in m, rad, m/s, rad/s).


# Arguments

- `parent`: Parent Object3D. If `parent` is present, the Object3D is defined relatively to `parent`.
  If `parent=nothing`, the Object3D is either the inertial system (typically called `world`), or the object is the reference system of a sub-system (via joints, all sub-systems must be connected directly or indirectly to `world`). Arguments `translation`, `rotation`, `velocity`, `angularVelocity` are ignored in this case (a warning is printed, if these arguments do not have zero values).

- `fixedToParent`: = true, if Object3D is fixed relatively to `parent`. Otherwise, Object3D can move freely relatively to parent (`translation`, `rotation`, `velocity`, `angularVelocity` is the initial state of Object3D with respect to `parent`). If `parent=nothing`, then `fixedToParent` is ignored.

- `translation`: Vector from the origin of the parent to the origin of the Object3D, resolved in the parent coordinate system. \\
  Example: `translation = [0.0, 0.5, 0.0]` or `translation = [0.0, 50.0, 0.0]u"cm"` are a relative translation of 0.5 m in y-direction of the parent.

- `rotation`: Vector `[angleX, angleY, angleZ]` to rotate the parent coordinate system along the x-axis with `angleX`, the y-axis with `angleY` and the z-axis with `angleZ` to arrive at the Object3D coordinate system.\\
  Example: `rotation = [0.0, pi/2, 0.0]` or `rotation = [0.0, 90u"°", 0.0]` defines that a rotation around the y-axis of the parent coordinate system with 90 degrees arrives at the Object3D.

- `velocity`: If `parent` is defined and `fixedToParent=false`, the initial velocity of the origin of the Object3D with respect to the `parent`, resolved in the parent coordinate system.\\
  Example: `velocity = [0.0, 0.5, 0.0]` or `velocity = [0.0, 50.0, 0.0]u"cm/s"` is an initial relative velocity of 0.5 m/s in y-direction of the parent.

- `angularVelocity`: If `parent` is defined and `fixedToParent=false`, the initial angular velocity of the Object3D with respect to the `parent`, resolved in the parent coordinate system.\\
  Example: `angularVelocity = [0.0, pi/2, 0.0]` or `angularVelocity = [0.0, 90u"°/s", 0.0]` is an initial relative angular velocity of 90 degrees per second in y-direction of the parent.

- `feature`: Defines the (optional) property associated with the Object3D by a constructor call. Supported constructors:
    - `Scene`: A [Scene](@ref) feature marks the root Object3D (world, origin, inertial system). It has no parent Object3D and allows to define global properties, such as the gravity field.
    - `Visual`: A [Visual](@ref) feature defines a shape used for visualization.
    - `Solid`: A [Solid](@ref) feature defines the solid properties of an Object3D, like mass, inertia tensor, collision behavior.
    - `nothing`: No feature is associated with the Object3D. This might be useful for helper Object3Ds, for example to mark a point on a shape and connecting it later via a joint.

# States, if freely moving

If `fixedToParent=false`, the Object3D is moving freely relatively to `parent`.
Vectors `translation`, `rotation`, `velocity`, `angularVelocity` (all resolved in `parent`) are used as states and are available in the result for plotting.

If `rotation[2]` is close to its singular position (= 90u"°" or -90u"°"), an event is triggered and the rotation sequence is changed from `[angleX, angleY, angleZ]` to
`[angleX, angleZ, angleY]`. In the new rotation sequence, `rotation[2]` is far from its singular position at this time instant. Variable `rotation123::Bool` in the result
signals whether `rotation` is defined with rotation sequence `[angleX, angleY, angleZ]` (`rotation123=true`) or with rotation sequence `[angleX, angleZ, angleY]` (`rotation123=false`).
See, example `Modia3D/test/Basic/ShaftFreeMotionAdaptiveRotSequence.jl`.


# Example

```julia
using Modia3D

model = Model3D(
    world      = Object3D(feature = Scene()),
    worldFrame = Object3D(parent  = :world,
                          feature = Visual(shape=CoordinateSystem())),
    support    = Object3D(parent  = :world, translation = [0.0, 0.5, 0.0],
                          feature = Visual(shape = Sphere(diameter=0.1)))
)
```
"""
mutable struct Object3D{F <: Modia3D.VarFloatType} <: Modia3D.AbstractObject3D
    # Tree connection structure of Object3D (for efficient processing of Object3Ds)
    parent::Object3D{F}                 # Parent Object3D (if parent===Object3D, no parent is yet defined)
    children::Vector{Object3D{F}}       # All Object3Ds, where Object3D is the parent
    isRootObj::Bool                  # = true, if it is a root obj of a super obj
    interactionManner::InteractionManner

    # Joint properties, defining the relative motion from parent to Object3D
    joint::Modia3D.AbstractJoint     # ::Fix, ::Revolute, ...

    # Efficient access of joint properties
    jointKind::JointKind             # Kind of joint
    jointIndex::Int                  # scene.<jointType>[jointIndex] = joint
    ndof::Int                        # Number of degrees of freedom
    canCollide::Bool                 # = false, if no collision parent/Object3D
    r_rel::SVector{3,F}        # Relative position vector from frame of parent Object3D to origin of Object3D frame, resolved in parent frame in [m]
                                     # (if parent===Object3D, r_rel=Modia3D.ZeroVector3D(F))
    R_rel::SMatrix{3,3,F,9}    # Rotation matrix from frame of parent Object3D to Object3D frame.
    r_abs::SVector{3,F}        # Absolute position vector from origin of world frame to origin of Object3D frame, resolved in world frame in [m]
    R_abs::SMatrix{3,3,F,9}    # Absolute rotation matrix from world frame to Object3D frame
    v0::SVector{3,F}           # Absolute velocity of Object3D origin, resolved in world frame (= der(r_abs)) in [m/s]
    w::SVector{3,F}            # Absolute angular velocity of Object3D, resolved in Object3D in [rad/s]
    a0::SVector{3,F}           # Absolute acceleration of Object3D origin, resolved in world frame (= der(v0)) in [m/s^2]
    z::SVector{3,F}            # Absolute angular acceleration of Object3D, resolved in Object3D in [rad/s^2]
    f::SVector{3,F}            # Cut-force resolved in Object3D in [N]
    t::SVector{3,F}            # Cut-torque resolved in Object3D in [N*m]

    # Mass properties.
    #   The root of each super object has potentially hasMass=true. All other Object3Ds have hasMass=false.
    #   The initial (fixed) mass properties defined in the Modia model are stored in feature.
    hasMass::Bool                 # = false, if m and I_CM are zero. = true, otherwise.
    m::F                    # Mass in [kg]
    r_CM::SVector{3,F}      # Position vector from Object3D to Center of Mass resolved in Object3D in [m]
    I_CM::SMatrix{3,3,F,9}  # Inertia matrix at Center of Mass resolved in a frame in the center of mass that is parallel to Object3D in [kg*m^2]

    # Additional information associated with Object3D
    feature::Union{Modia3D.AbstractObject3DFeature, Modia3D.AbstractScene}  # Optional feature associated with Object3D
    twoObject3Dobject::Vector{Modia3D.AbstractTwoObject3DObject}  # Optional AbstractTwoObject3DObject object associated with Object3D
    hasCutJoint::Bool          # = true if it has a cut joint
    hasForceElement::Bool      # = true if it has a force element
    hasChildJoint::Bool        # = true if its child has a joint
    computeAcceleration::Bool  # = true if acceleration needs to be computed

    # internal shortcuts to avoid costly runtime dispatch
    shapeKind::Shapes.ShapeKind             # marks the defined shape
    shape::Modia3D.AbstractShape            # stores shape defined in Solid or Visual
    visualMaterial::Shapes.VisualMaterial   # stores visualMaterial defined in Solid or Visual
    centroid::SVector{3,F}            # stores the centroid of a solid shape

    # = True: Coordinate system of Object3D is always visualized
    # = False: Coordinate system of Object3D is never visualized
    # = Inherited: Coordinate system of Object3D is visualized, if Scene(visualizeFrames=true)
    visualizeFrame::Modia3D.Ternary
    visualizationFrame::Vector{Object3D{F}} # If to be visualized, the Object3D holding the coordinate system.

    # additional 3D Shapes
    fileMeshConvexShapes::Vector{Object3D{F}} # a graphical decomposition of a 3D mesh must be stored additionally

    # if enabled, all AABBs are visualized and stored in world Object3D
    AABBVisu::Vector{Object3D{F}}
    # if enabled, contact and support points are set to world Object3D
    # for visualizing contact points (each contact has two points)
    contactVisuObj1::Vector{Object3D{F}}
    contactVisuObj2::Vector{Object3D{F}}
    # for visualizing support points (each contact has 6 support points, 3 on each collision obj)
    supportVisuObj1A::Vector{Object3D{F}}
    supportVisuObj2A::Vector{Object3D{F}}
    supportVisuObj3A::Vector{Object3D{F}}
    supportVisuObj1B::Vector{Object3D{F}}
    supportVisuObj2B::Vector{Object3D{F}}
    supportVisuObj3B::Vector{Object3D{F}}

    path::String


    ###--------------------- Object3D constructor ------------------------------
    ## Constructor 1
    # constructor for Modia interface (only keyword arguments)
    function Object3D{F}(;
        parent::Union{Object3D,Nothing} = nothing,
        path::String="",
        fixedToParent::Bool = true,
        translation     = Modia3D.ZeroVector3D(F),
        rotation        = Modia3D.ZeroVector3D(F),
        velocity        = Modia3D.ZeroVector3D(F),
        angularVelocity = Modia3D.ZeroVector3D(F),
        feature = nothing,
        angularVelocityResolvedInParent = true,
        kwargs...) where F <: Modia3D.VarFloatType
        interactionBehavior = Modia3D.NoInteraction

        if length(kwargs) > 0
            @warn "Object3D $path: Keyword arguments `$(kwargs...)` are ignored."
        end

        # create Object3D with created feature
        visualizeFrame = Modia3D.Inherited

        if isnothing(feature)
            feature = emptyObject3DFeature
        end
        if isnothing(translation)
            @warn("$path::Object3D: translation=nothing is interpreted as translation=[0.0, 0.0, 0.0]")
            translation = Modia3D.ZeroVector3D(F)
        end
        if isnothing(rotation)
            @warn("$path::Object3D: rotation=nothing is interpreted as rotation=[0.0, 0.0, 0.0]")
            rotation = Modia3D.ZeroVector3D(F)
        end
        if isnothing(velocity)
            @warn("$path::Object3D: velocity=nothing is interpreted as velocity=[0.0, 0.0, 0.0]")
            velocity = Modia3D.ZeroVector3D(F)
        end
        if isnothing(angularVelocity)
            @warn("$path::Object3D: angularVelocity=nothing is interpreted as angularVelocity=[0.0, 0.0, 0.0]")
            angularVelocity = Modia3D.ZeroVector3D(F)
        end

        if !isnothing(parent)
            # with parent

            if fixedToParent
                if !iszero(velocity) || !iszero(angularVelocity)
                    @warn("$path::Object3D is fixed to parent $(parent.path).\nTherefore, velocity=$velocity and angularVelocity=$angularVelocity are ignored.")
                end
            end

            translation = Modia3D.convertAndStripUnit(SVector{3,F}, u"m"  , translation)
            rotation    = Modia3D.convertAndStripUnit(SVector{3,F}, u"rad", rotation)

            if !fixedToParent
                velocity        = Modia3D.convertAndStripUnit(SVector{3,F}, u"m/s"  , velocity)
                angularVelocity = Modia3D.convertAndStripUnit(SVector{3,F}, u"rad/s", angularVelocity)
            end

            r_rel = translation
            r_abs = parent.r_abs + r_rel
            R_rel = Frames.rot123(rotation[1], rotation[2], rotation[3])
            R_abs = R_rel*parent.R_abs

            #obj = Object3D{F}(parent, feature, fixed=fixed, r=translation, R=rotation, v_start=velocity, w_start=angularVelocity, w_startVariables=w_startVariables, visualizeFrame=visualizeFrame, path=path)

            visualizeFrame2 = typeof(visualizeFrame) == Modia3D.Ternary ? visualizeFrame : (visualizeFrame ? Modia3D.True : Modia3D.False)

            (shapeKind, shape, visualMaterial, centroid) = setShapeKind(F, feature)
            obj = new(parent, Vector{Object3D{F}}[],
                false, InteractionManner(interactionBehavior), FixedJoint{F}(), FixKind, 0, 0, true,
                r_rel, R_rel, r_abs, R_abs,
                Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F),
                false, F(0.0), Modia3D.ZeroVector3D(F), SMatrix{3,3,F,9}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                feature, Modia3D.AbstractTwoObject3DObject[],
                false, false, false, false,
                shapeKind, shape, visualMaterial, centroid,
                visualizeFrame2,
                Vector{Object3D{F}}[],
                Vector{Object3D{F}}[], Vector{Object3D{F}}[],
                Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[],
                Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[],
                path)

            if fixedToParent
                # obj is children of parent
                push!(parent.children, obj)

            else
                # obj has FreeMotion joint
                FreeMotion{F}(; obj1=parent, obj2=obj, path=path, r=translation, rot=rotation, v=velocity, w=angularVelocity, hiddenStates=true,
                                wResolvedInParent=angularVelocityResolvedInParent)
            end

        else
            # no parent
            if !iszero(translation) || !iszero(rotation) || !iszero(velocity) || !iszero(angularVelocity)
                @warn("$path::Object3D has parent=nothing.\nTherefore, translation = $translation, rotation = $rotation,\nvelocity = $velocity and angularVelocity = $angularVelocity are ignored.")
            end

            # obj = Object3D{F}(feature, visualizeFrame=visualizeFrame, path=path)
            obj = Object3DWithoutParent(new(), visualizeFrame = visualizeFrame, interactionBehavior = interactionBehavior, path=path)
            obj.feature = feature
            (obj.shapeKind, obj.shape, obj.visualMaterial, obj.centroid) = setShapeKind(F, feature)
        end

        if typeof(feature) <: Modia3D.Shapes.Visual && typeof(feature.shape) <: Modia3D.Shapes.FileMesh && feature.shape.convexPartition
            createConvexPartition(obj, feature, feature.shape)
        end

        if typeof(feature) <: Modia3D.Shapes.Solid && typeof(feature.shape) <: Modia3D.Shapes.FileMesh && feature.shape.convexPartition
            createConvexPartition(obj, feature, feature.shape)
        end

        return obj
    end

    ###--------------------- Object3D constructor ------------------------------
    ## Constructor 2
    # Object3D constructor: with feature and without parent (less keywords)
    function Object3D{F}(feature;
                    visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited, interactionBehavior::InteractionBehavior = Modia3D.NoInteraction,
                    path::String="")::Object3D where F <: Modia3D.VarFloatType
        obj = Object3DWithoutParent( new(), visualizeFrame = visualizeFrame, interactionBehavior = interactionBehavior, path=path)

        obj.feature = feature
        (obj.shapeKind, obj.shape, obj.visualMaterial, obj.centroid) = setShapeKind(F, feature)
        return obj
    end

    ###--------------------- Object3D constructor ------------------------------
    ## Constructor 3
    # Object3D constructor: with feature and with parent
    function Object3D{F}(parent::Object3D{F},
                      feature::Modia3D.AbstractObject3DFeature = emptyObject3DFeature;
                      fixed::Bool = true,
                      interactionBehavior::InteractionBehavior = Modia3D.NoInteraction,
                      r::AbstractVector = Modia3D.ZeroVector3D(F),
                      R::Union{SMatrix{3,3,F,9},Nothing} = nothing,
                      q::Union{SVector{4,F},Nothing} = nothing,
                      v_start::AbstractVector = Modia3D.ZeroVector3D(F),
                      w_start::AbstractVector = Modia3D.ZeroVector3D(F),
                      w_startVariables::WStartVariables = WCartesian,
                      visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited,
                      path::String="")::Object3D where F <: Modia3D.VarFloatType

        if !isnothing(R) && !isnothing(q)
            error("Modia3D.Composition.Object3D: either R or q must be nothing but both have a value.")
        end
        if !isnothing(R)
            Modia3D.assertRotationMatrix(R)
        elseif !isnothing(q)
            Modia3D.assertQuaternion(q)
        end

        r_rel = SVector{3,F}(r)
        R_rel = !isnothing(R) ? R : (!isnothing(q) ? Modia3D.from_q(q) : Modia3D.NullRotation(F))
        r_abs = parent.r_abs + r_rel
        R_abs = R_rel*parent.R_abs

        visualizeFrame2 = typeof(visualizeFrame) == Modia3D.Ternary ? visualizeFrame : (visualizeFrame ? Modia3D.True : Modia3D.False)

        (shapeKind, shape, visualMaterial, centroid) = setShapeKind(F, feature)
        obj = new(parent, Vector{Object3D{F}}[],
              false, InteractionManner(interactionBehavior), FixedJoint{F}(), FixKind, 0, 0, true,
              r_rel, R_rel, r_abs, R_abs,
              Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F),
              false, F(0.0), Modia3D.ZeroVector3D(F), SMatrix{3,3,F,9}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
              feature, Modia3D.AbstractTwoObject3DObject[],
              false, false, false, false,
              shapeKind, shape, visualMaterial, centroid,
              visualizeFrame2,
              Vector{Object3D{F}}[],
              Vector{Object3D{F}}[], Vector{Object3D{F}}[],
              Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[],
              Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[],
              path)


        if fixed
            push!(parent.children, obj)

        else
            # FreeMotionCardan pushes obj on parent.children
            rot_start = !isnothing(R) ? rot123fromR(R) : !isnothing(q) ? rot123fromR(Modia3D.from_q(q)) : Modia3D.ZeroVector3D(F)
            if w_startVariables == WCardan123
                # convert Cardan123 -> Cartesian
                om_start = wfromrot123(rot_start, w_start)
            else
                om_start = w_start
            end
            FreeMotionCardan(; r   = r_rel,
                               rot = rot_start,
                               v   = SVector{3,F}(v_start),
                               w   = SVector{3,F}(om_start) )
        end

        return obj
    end

    ###--------------------- Object3D constructor -----------------------------
    ## Constructor 4
    # Constructor used only for internal purposes (not to be directly used by the user)
    # used by e.g. copyObject3D
    function Object3D{F}(
            parent::Object3D{F}, r_rel::SVector{3,F},
            R_rel::SMatrix{3,3,F,9}, r_abs::SVector{3,F},
            R_abs::SMatrix{3,3,F,9}, feature::Modia3D.AbstractObject3DFeature,
            visualizeFrame::Modia3D.Ternary, path::String="") where F <: Modia3D.VarFloatType

        (shapeKind, shape, visualMaterial, centroid) = setShapeKind(F, feature)
        new(parent, Vector{Object3D{F}}[], false,
        InteractionManner(Modia3D.NoInteraction), FixedJoint{F}(), FixKind, 0, 0, true,
        r_rel, R_rel, r_abs, R_abs,
        Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F),
        false, F(0.0), Modia3D.ZeroVector3D(F), SMatrix{3,3,F,9}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        feature, Modia3D.AbstractTwoObject3DObject[],
        false, false, false, false,
        shapeKind, shape, visualMaterial, centroid,
        visualizeFrame,
        Vector{Object3D{F}}[],
        Vector{Object3D{F}}[], Vector{Object3D{F}}[],
        Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[],
        Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[], Vector{Object3D{F}}[],
        path)
    end
end

Object3D(args... ; kwargs...) = Object3D{Float64}(args... ; kwargs...)

function setShapeKind(::Type{F}, feature) where F <: Modia3D.VarFloatType
    if typeof(feature) <: Modia3D.Shapes.Solid || typeof(feature) <: Modia3D.Shapes.Visual
        shapeKind = Modia3D.getShapeKind(feature.shape)
        shape = feature.shape

        centroid = Modia3D.ZeroVector3D(F)
        if typeof(feature) <: Modia3D.Shapes.Solid && !isnothing(shape)
            centroid = Modia3D.centroid(shape)
        end

        if shapeKind == Modia3D.UndefinedShapeKind
            shape = Modia3D.Shapes.Sphere{F}()
        end
        visualMaterial = feature.visualMaterial
        if isnothing(visualMaterial)
            visualMaterial = Modia3D.Shapes.VisualMaterial()
            @warn("No visualMaterial defined for ", shape)
        end
        return shapeKind, shape, visualMaterial, centroid
    else
        return Modia3D.UndefinedShapeKind, Modia3D.Shapes.Sphere{F}(), Modia3D.Shapes.VisualMaterial(), Modia3D.ZeroVector3D(F)
    end
end

# Object3DWithoutParent is called from constructor 1 or 2 (for objs without a parent)
function Object3DWithoutParent(obj::Object3D{F};
                               visualizeFrame::Union{Modia3D.Ternary,Bool} = Modia3D.Inherited, interactionBehavior::InteractionBehavior = Modia3D.NoInteraction,
                               path::String="") where F <: Modia3D.VarFloatType
    obj.parent             = obj
    obj.children           = Vector{Object3D{F}}[]
    obj.isRootObj          = false
    obj.interactionManner  = InteractionManner(interactionBehavior)
    obj.joint              = FixedJoint{F}()
    obj.jointKind          = FixKind
    obj.jointIndex         = 0
    obj.ndof               = 0
    obj.canCollide         = true
    obj.r_rel              = Modia3D.ZeroVector3D(F)
    obj.R_rel              = Modia3D.NullRotation(F)
    obj.r_abs              = Modia3D.ZeroVector3D(F)
    obj.R_abs              = Modia3D.NullRotation(F)
    obj.v0                 = Modia3D.ZeroVector3D(F)
    obj.w                  = Modia3D.ZeroVector3D(F)
    obj.a0                 = Modia3D.ZeroVector3D(F)
    obj.z                  = Modia3D.ZeroVector3D(F)
    obj.f                  = Modia3D.ZeroVector3D(F)
    obj.t                  = Modia3D.ZeroVector3D(F)
    obj.hasMass            = false
    obj.m                  = F(0.0)
    obj.r_CM               = Modia3D.ZeroVector3D(F)
    obj.I_CM               = SMatrix{3,3,F,9}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    obj.twoObject3Dobject  = Modia3D.AbstractTwoObject3DObject[]
    obj.hasCutJoint        = false
    obj.hasForceElement    = false
    obj.hasChildJoint      = false
    obj.computeAcceleration = false
    obj.visualizeFrame     = typeof(visualizeFrame) == Modia3D.Ternary ? visualizeFrame : (visualizeFrame ? Modia3D.True : Modia3D.False)
    obj.visualizationFrame = Vector{Object3D{F}}[]
    obj.fileMeshConvexShapes = Vector{Object3D{F}}[]
    obj.AABBVisu           = Vector{Object3D{F}}[]
    obj.contactVisuObj1    = Vector{Object3D{F}}[]
    obj.contactVisuObj2    = Vector{Object3D{F}}[]
    obj.supportVisuObj1A   = Vector{Object3D{F}}[]
    obj.supportVisuObj2A   = Vector{Object3D{F}}[]
    obj.supportVisuObj3A   = Vector{Object3D{F}}[]
    obj.supportVisuObj1B   = Vector{Object3D{F}}[]
    obj.supportVisuObj2B   = Vector{Object3D{F}}[]
    obj.supportVisuObj3B   = Vector{Object3D{F}}[]
    obj.path               = path
    return obj
end

# addObject3DVisualizationFrame! calls constructor 4
#  - internally used for visualizing a frame by a coordinate system
#    its called: "name of obj" + ".visualizationFrame"
function addObject3DVisualizationFrame!(obj::Object3D{F},
                                        feature::Modia3D.AbstractObject3DFeature, name) where F <: Modia3D.VarFloatType
    push!(obj.visualizationFrame, Object3D{F}(obj.parent,
                                    obj.r_rel, obj.R_rel, obj.r_abs,
                                    obj.R_abs, feature, obj.visualizeFrame) )
end


#    its called: "name of obj" + ".mesh[i]"
function createConvexPartition(obj::Object3D{F}, feature, mesh) where F <: Modia3D.VarFloatType # feature Visual
    (head,ext) = splitext(mesh.filename)
    if ext == ".obj" && mesh.convexPartition
        convexDecompositionDirectory = joinpath(dirname(mesh.filename),"convexSolids_" * basename(mesh.filename))
        contentDir = readdir(convexDecompositionDirectory)
        # names = Vector{AbstractString}()
        i = 1
        for name in contentDir
            path = String(Symbol(obj.path, ".", "mesh", i ,))
            (head,ext) = splitext(name)
            if ext == ".obj"
                fileMesh = Modia3D.Shapes.FileMesh(filename = joinpath(convexDecompositionDirectory, name),
                scale = mesh.scaleFactor, useMaterialColor = mesh.useMaterialColor, smoothNormals = mesh.smoothNormals, convexPartition = false)

                feature = createFileFeature(feature, fileMesh)

                push!(obj.fileMeshConvexShapes, Object3D{F}(obj, feature, path = path) )
                i = i + 1
            end
        end
    end
    return nothing
end

function createFileFeature(feature::Shapes.Visual, fileMesh)
    return Modia3D.Shapes.Visual(shape = fileMesh, visualMaterial = feature.visualMaterial)
end

function createFileFeature(feature::Shapes.Solid{F}, fileMesh) where F <: Modia3D.VarFloatType
    return Modia3D.Shapes.Solid{F}(shape=fileMesh, massProperties=nothing, solidMaterial=feature.solidMaterial, collision=feature.collision, contactMaterial=feature.contactMaterial, collisionSmoothingRadius=feature.collisionSmoothingRadius, visualMaterial=feature.visualMaterial)
end

function addAABBVisuToWorld!(world::Object3D{F}, AABB::Vector{Vector{Basics.BoundingBox{F}}}) where F <: Modia3D.VarFloatType
    k = 0
    @inbounds for i = 1:length(AABB)
        for j = 1:length(AABB[i])
            k = k + 1
            name = String(Symbol(world.path, ".", "AABBVisu", i, j))
            aabb = AABB[i][j]
            feature = Modia3D.Shapes.Visual(shape = Modia3D.Shapes.Box{F}(
                    lengthX = abs(aabb.x_max - aabb.x_min), lengthY = abs(aabb.y_max - aabb.y_min), lengthZ = abs(aabb.z_max - aabb.z_min)),
                visualMaterial = Modia3D.Shapes.VisualMaterial(color="grey96", transparency=0.8))
            push!(world.AABBVisu,  Object3D{F}(world, feature, path = name) )
        end
    end

end

function addContactVisuObjToWorld!(world::Object3D{F}, nVisualContSupPoints, defaultContactSphereDiameter) where F <: Modia3D.VarFloatType
    world.contactVisuObj1 = fill(Object3D{F}(), nVisualContSupPoints)
    world.contactVisuObj2 = fill(Object3D{F}(), nVisualContSupPoints)
    @inbounds for i = 1:length(world.contactVisuObj1)
        name1 = String(Symbol(world.path, ".", "contactVisuObj1", i))
        name2 = String(Symbol(world.path, ".", "contactVisuObj2", i))

        feature1 = Modia3D.Shapes.Visual(shape = Modia3D.Shapes.Sphere{F}(diameter = defaultContactSphereDiameter), visualMaterial = Modia3D.Shapes.VisualMaterial(color="Red",   transparency=1.0))
        feature2 = Modia3D.Shapes.Visual(shape = Modia3D.Shapes.Sphere{F}(diameter = defaultContactSphereDiameter), visualMaterial = Modia3D.Shapes.VisualMaterial(color="Black",   transparency=1.0))

        world.contactVisuObj1[i] =  Object3D{F}(world, feature1, path = name1)
        world.contactVisuObj2[i] =  Object3D{F}(world, feature2, path = name2)
    end
end

function addSupportVisuObjToWorld!(world::Object3D{F}, nVisualContSupPoints, defaultContactSphereDiameter) where F <: Modia3D.VarFloatType
    world.supportVisuObj1A = fill(Object3D{F}(), nVisualContSupPoints)
    world.supportVisuObj2A = fill(Object3D{F}(), nVisualContSupPoints)
    world.supportVisuObj3A = fill(Object3D{F}(), nVisualContSupPoints)
    world.supportVisuObj1B = fill(Object3D{F}(), nVisualContSupPoints)
    world.supportVisuObj2B = fill(Object3D{F}(), nVisualContSupPoints)
    world.supportVisuObj3B = fill(Object3D{F}(), nVisualContSupPoints)
    @inbounds for i = 1:length(world.supportVisuObj1A)
        name1 = String(Symbol(world.path, ".", "supportVisuObj1A", i))
        name2 = String(Symbol(world.path, ".", "supportVisuObj2A", i))
        name3 = String(Symbol(world.path, ".", "supportVisuObj3A", i))
        name4 = String(Symbol(world.path, ".", "supportVisuObj1B", i))
        name5 = String(Symbol(world.path, ".", "supportVisuObj2B", i))
        name6 = String(Symbol(world.path, ".", "supportVisuObj3B", i))

        featureA = Modia3D.Shapes.Visual(shape = Modia3D.Shapes.Sphere{F}(diameter = defaultContactSphereDiameter), visualMaterial = Modia3D.Shapes.VisualMaterial(color="Red", transparency=1.0))

        featureB = Modia3D.Shapes.Visual(shape = Modia3D.Shapes.Sphere{F}(diameter = defaultContactSphereDiameter), visualMaterial = Modia3D.Shapes.VisualMaterial(color="Black", transparency=1.0))

        world.supportVisuObj1A[i] = Object3D{F}(world, featureA, path = name1)
        world.supportVisuObj2A[i] = Object3D{F}(world, featureA, path = name2)
        world.supportVisuObj3A[i] = Object3D{F}(world, featureA, path = name3)
        world.supportVisuObj1B[i] = Object3D{F}(world, featureB, path = name4)
        world.supportVisuObj2B[i] = Object3D{F}(world, featureB, path = name5)
        world.supportVisuObj3B[i] = Object3D{F}(world, featureB, path = name6)
    end
end


# Inquire properties of a Object3D
fullName(             obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.path # Modia.SimulationModel.modelName + Object3D.path (wird ca. 98x verwendet)
instanceName(         obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.path
fullName(obj) = error("fullName not implemented for ", typeof(obj))
instanceName(obj) = error("instanceName not implemented for ", typeof(obj))
hasParent(            obj::Object3D{F}) where F <: Modia3D.VarFloatType = !(obj.parent === obj)
hasNoParent(          obj::Object3D{F}) where F <: Modia3D.VarFloatType =   obj.parent === obj
hasChildren(          obj::Object3D{F}) where F <: Modia3D.VarFloatType = length(obj.children) > 0
hasNoChildren(        obj::Object3D{F}) where F <: Modia3D.VarFloatType = length(obj.children) == 0
isWorld(              obj::Object3D{F}) where F <: Modia3D.VarFloatType = hasNoParent(obj) && typeof(obj.feature) <: Modia3D.Composition.Scene
isNotWorld(           obj::Object3D{F}) where F <: Modia3D.VarFloatType = !(isWorld(obj))
isMovable(            obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.interactionManner.movable
isLockable(           obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.interactionManner.lockable
isFixed(              obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.ndof == 0
isNotFixed(           obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.ndof > 0
isFree(               obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.ndof == 6
isNotFree(            obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.ndof < 6
hasJoint(             obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.ndof > 0 && obj.ndof < 6
hasNoJoint(           obj::Object3D{F}) where F <: Modia3D.VarFloatType = isFixed(obj) || isFree(obj)
isCoordinateSystem(   obj::Object3D{F}) where F <: Modia3D.VarFloatType = typeof(obj.feature) == Shapes.CoordinateSystem
isNotCoordinateSystem(obj::Object3D{F}) where F <: Modia3D.VarFloatType = typeof(obj.feature) != Shapes.CoordinateSystem
hasForceElement(      obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.hasForceElement
hasChildJoint(        obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.hasChildJoint
needsAcceleration(    obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.computeAcceleration
objectHasMass(        obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.hasMass
isRootObject(         obj::Object3D{F}) where F <: Modia3D.VarFloatType = obj.isRootObj
objectHasMovablePos(  obj::Object3D{F}) where F <: Modia3D.VarFloatType = !isnothing(obj.interactionManner.movablePos)

featureHasMass(obj::Object3D{F}) where F <: Modia3D.VarFloatType = featureHasMass(obj.feature)
featureHasMass(feature::Modia3D.AbstractObject3DFeature) = false
featureHasMass(feature::Modia3D.AbstractScene) = false
featureHasMass(feature::Shapes.Solid)                 = !isnothing(feature.massProperties)

isVisible(obj::Object3D, renderer::Modia3D.AbstractRenderer) = isVisible(obj.feature, renderer)
isVisible(feature::Modia3D.AbstractObject3DFeature, renderer::Modia3D.AbstractRenderer) = false

isVisible(feature::Modia3D.AbstractScene, renderer::Modia3D.AbstractRenderer) = false

canCollide(feature::Modia3D.AbstractObject3DFeature) = false

function canCollide(obj::Object3D)::Bool
    if typeof(obj.feature) <: Modia3D.Shapes.Solid
        if !isnothing(obj.feature.shape) && !isnothing(obj.feature.contactMaterial)
            if typeof(obj.feature.contactMaterial) == String
                return obj.feature.contactMaterial != ""
            else
                return true
    end; end; end
    return false
end


"""    rootObject3D(obj) - returns the root Object3D of all parents of obj"""
function rootObject3D(obj::Object3D{F}) where F <: Modia3D.VarFloatType
    obj1 = obj
    while hasParent(obj1)
        obj1 = obj1.parent
    end
    return obj1
end


"""    path = rootObject3DPath(obj) - returns a vector of Object3D names of all objects from obj to root"""
function rootObject3DPath(obj::Object3D{F}) where F <: Modia3D.VarFloatType
    path = String[]
    obj1 = obj
    while hasParent(obj1)
        obj1 = obj1.parent
        push!(path, obj1.path)
    end
    return path
end


"""    removeChild!(obj, child) - Remove child from obj.children"""
function removeChild!(obj::Object3D{F}, child::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    children = obj.children
    for i in eachindex(children)
        if children[i] === child
            deleteat!(children,i)
            return nothing
    end; end
    error("\nError from Modia3D.Composition.removeChild!(", obj, ", ", child, ")\n",
            child, " is not a child of object ", obj)
    return nothing
end


# Print Object3D
function Base.show(io::IO, obj::Object3D)
    print(io,"Object3D(path=", obj.path)
    commaNeeded = true
    if hasParent(obj)
        if commaNeeded
            print(io,", ")
        end
        print(io, "parent=", obj.parent.path)
    end
    if hasJoint(obj)
        if commaNeeded
            print(io,", ")
        end
        print(io, "joint=", typeof(obj.joint), "(path = \"", obj.joint.path, "\", ...)")
    end
    if commaNeeded
        print(io,", ")
    end
    print(io, "feature=", typeof(obj.feature), "(...))")
end
