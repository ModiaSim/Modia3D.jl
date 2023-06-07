# Modia interface to Modia3D
Model3D(; kwargs...) = Model(; _buildFunction       = Par(functionName = :(Modia3D.build_Model3D!)),
                               _initSegmentFunction = Par(functionName = :(Modia3D.initSegment_Model3D!)), kwargs...)

Object3D(             ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.Object3D{FloatType})             , _path = true, kwargs...)
Scene(                ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.Scene{FloatType})                              , kwargs...)
Visual(               ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Visual)                                             , kwargs...)
Solid(                ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Solid{FloatType})                                   , kwargs...)
Box(                  ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Box{FloatType})                                     , kwargs...)
Beam(                 ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Beam{FloatType})                                    , kwargs...)
Cylinder(             ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Cylinder{FloatType})                                , kwargs...)
Sphere(               ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Sphere{FloatType})                                  , kwargs...)
Ellipsoid(            ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Ellipsoid{FloatType})                               , kwargs...)
Capsule(              ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Capsule{FloatType})                                 , kwargs...)
Cone(                 ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Cone{FloatType})                                    , kwargs...)
SpringShape(          ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Spring)                                             , kwargs...)
GearWheel(            ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.GearWheel)                                          , kwargs...)
Grid(                 ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Grid)                                               , kwargs...)
VisualMaterial(       ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.VisualMaterial)                                     , kwargs...)
MassProperties(       ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.MassProperties{FloatType})                          , kwargs...)
CoordinateSystem(     ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.CoordinateSystem)                                   , kwargs...)
FileMesh(             ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.FileMesh)                                           , kwargs...)
Font(                 ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.Font)                                               , kwargs...)
TextShape(            ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.TextShape)                                          , kwargs...)
ModelicaShape(        ; kwargs...) = Par(; _constructor = :(Modia3D.Shapes.ModelicaShape)                                      , kwargs...)
Fix(                  ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.Fix{FloatType})                                , kwargs...)
Free(                 ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.Free{FloatType})                               , kwargs...)
WorldForce(           ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.WorldForce{FloatType})           , _path = true, kwargs...)
WorldTorque(          ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.WorldTorque{FloatType})          , _path = true, kwargs...)
Bushing(              ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.Bushing{FloatType})              , _path = true, kwargs...)
SpringDamperPtP(      ; kwargs...) = Par(; _constructor = :(Modia3D.Composition.SpringDamperPtP{FloatType})      , _path = true, kwargs...)
PolygonalContactModel(; kwargs...) = Par(; _constructor = :(Modia3D.Composition.PolygonalContactModel{FloatType}), _path = true, kwargs...)

MassPropertiesFromShape()              = Par(; _constructor = :(Modia3D.Shapes.MassPropertiesFromShape{FloatType}))
MassPropertiesFromShapeAndMass(; mass) = Par(; _constructor = :(Modia3D.Shapes.MassPropertiesFromShapeAndMass{FloatType}), mass = mass)
UniformGravityField(; kwargs...)       = Par(; _constructor = :(Modia3D.Composition.UniformGravityField), kwargs...)

# Interface functions needed for path planning, and model actions
ModelActions(; kwargs...) = Par(; _constructor = :(Modia3D.PathPlanning.ModelActions{FloatType,FloatType}), _path = true, _instantiatedModel=true, kwargs...)
ActionAttach(args...; kwargs...) = Modia3D.PathPlanning.ActionAttach(args...; kwargs...)
ActionRelease(args...; kwargs...) = Modia3D.PathPlanning.ActionRelease(args...; kwargs...)
ActionReleaseAndAttach(args...; kwargs...) = Modia3D.PathPlanning.ActionReleaseAndAttach(args...; kwargs...)
ActionDelete(args...; kwargs...) = Modia3D.PathPlanning.ActionDelete(args...; kwargs...)
EventAfterPeriod(args...; kwargs...) = Modia3D.PathPlanning.EventAfterPeriod(args...; kwargs...)
ActionWait(args...; kwargs...) = Modia3D.PathPlanning.ActionWait(args...; kwargs...)
addReferencePath(args...; kwargs...) = Modia3D.PathPlanning.addReferencePath(args...; kwargs...)
ptpJointSpace(args...; kwargs...)           = Modia3D.PathPlanning.ptpJointSpace(args...; kwargs...)
getRefPathPosition(args...)          = Modia3D.PathPlanning.getRefPathPosition(args...)
getRefPathInitPosition(args...)      = Modia3D.PathPlanning.getRefPathInitPosition(args...)
executeActions(args...)      = Modia3D.PathPlanning.executeActions(args...)


Revolute(; obj1, obj2, axis=3, phi=Var(init=0.0), w=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Composition.Revolute{FloatType}), _path = true, _jointType = :Revolute),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    axis = Par(value = axis),
    canCollide = Par(value = canCollide),
    phi  = phi,
    w    = w,
    equations = :[
        w = der(phi)
        ]
)

RevoluteWithFlange(; obj1, obj2, axis=3, phi=Var(init=0.0), w=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Composition.Revolute{FloatType}), _path = true, _jointType = :RevoluteWithFlange),
    obj1   = Par(value = obj1),
    obj2   = Par(value = obj2),
    axis   = Par(value = axis),
    canCollide = Par(value = canCollide),
    flange = Flange,
    phi    = phi,
    w      = w,
    equations = :[
        phi = flange.phi
        w   = der(phi)
        ]
)

Prismatic(; obj1, obj2, axis=1, s=Var(init=0.0), v=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Composition.Prismatic{FloatType}), _path = true, _jointType = :Prismatic),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    axis = Par(value = axis),
    canCollide = Par(value = canCollide),
    s    = s,
    v    = v,
    equations = :[
        v = der(s)
        ]
)

PrismaticWithFlange(; obj1, obj2, axis=1, s=Var(init=0.0), v=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Composition.Prismatic{FloatType}), _path = true, _jointType = :PrismaticWithFlange),
    obj1   = Par(value = obj1),
    obj2   = Par(value = obj2),
    axis   = Par(value = axis),
    canCollide = Par(value = canCollide),
    flange = TranslationalFlange,
    s      = s,
    v      = v,
    equations = :[
        s = flange.s
        v = der(s)
        ]
)


"""
    next_isrot123 = change_rotSequenceInNextIteration!(rot::AbstractVector, isrot123::Bool, instantiatedModel::InstantiatedModel, x, rot_name)

Change rotation sequence of `rot` from `x-axis, y-axis, z-axis` to `x-axis, z-axis, y-axis` or visa versa in the next event iteration:

- If `isrot123 = true`, return `next_isrot123 = false` and `x[..] = rot132fromR(Rfromrot123(rot))`

- If `isrot123 = false`, return `next_isrot123 = true` and `x[..] = rot123fromR(Rfromrot132(rot))`
"""
function change_rotSequenceInNextIteration!(rot::AbstractVector, isrot123::Bool, instantiatedModel::InstantiatedModel, x, rot_name)::Bool
    if isrot123
        #println("        switch $rot_name 123 -> 132")
        next_rot      = Modia3D.rot132fromR(Modia3D.Rfromrot123(rot))
        next_isrot123 = false
    else
        #println("        switch $rot_name 132 -> 123")
        next_rot      = Modia3D.rot123fromR(Modia3D.Rfromrot132(rot))
        next_isrot123 = true
    end

    # Change x-vector with the next_rot values
    eqInfo = instantiatedModel.equationInfo
    startIndex = eqInfo.x_info[ eqInfo.x_dict[rot_name] ].startIndex
    x[startIndex]   = next_rot[1]
    x[startIndex+1] = next_rot[2]
    x[startIndex+2] = next_rot[3]
    return next_isrot123
end

singularRem(ang) = abs(rem2pi(ang, RoundNearest)) - 1.5  # is negative/positive in valid/singular angle range

FreeMotion(; obj1, obj2, r=Var(init=Modia.SVector{3,Float64}(zeros(3))), rot=Var(init=Modia.SVector{3,Float64}(zeros(3))), v=Var(init=Modia.SVector{3,Float64}(zeros(3))), w=Var(init=Modia.SVector{3,Float64}(zeros(3)))) = Model(; _constructor = Par(value = :(Modia3D.Composition.FreeMotion{FloatType}), _path = true, _jointType = :FreeMotion),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    r    = r,
    rot  = rot,
    v    = v,
    w    = w,

    next_isrot123 = Var(start=true),
    _rotName = "???",  # is changed by buildModia3D to the full path name of "rot"

    equations = :[
        der(r) = v

        isrot123 = pre(next_isrot123)
        rot2_singularity = positive(singularRem(rot[2]))
        next_isrot123 = if rot2_singularity; change_rotSequenceInNextIteration!(rot, isrot123, instantiatedModel, _x, _rotName) else isrot123 end
        der(rot) = Modia3D.J123or132(rot,isrot123) * w
        ]
)

FreeMotion2(; obj1, obj2, r=Var(init=Modia.SVector{3,Float64}(zeros(3))), rot=Var(init=Modia.SVector{3,Float64}(zeros(3))), v=Var(init=Modia.SVector{3,Float64}(zeros(3))), w=Var(init=Modia.SVector{3,Float64}(zeros(3)))) = Model(; _constructor = Par(value = :(Modia3D.Composition.FreeMotion{FloatType}), _path = true, _jointType = :FreeMotion),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    r    = r,
    rot  = rot,
    v    = v,
    w    = w,
    equations = :[
        der(r) = v
        isrot123 = true
        der(rot) = Modia3D.J123or132(rot,true) * w
        ]
)
