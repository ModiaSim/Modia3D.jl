# ModiaLang interface to Modia3D
Object3D(        ; kwargs...) = Par(; _constructor = :(Modia3D.Object3D), _path = true, kwargs...)
Scene(           ; kwargs...) = Par(; _constructor = :(Modia3D.SceneOptions)          , kwargs...)
Visual(          ; kwargs...) = Par(; _constructor = :(Modia3D.Visual)                , kwargs...)
Solid(           ; kwargs...) = Par(; _constructor = :(Modia3D.Solid)                 , kwargs...)
Box(             ; kwargs...) = Par(; _constructor = :(Modia3D.Box)                   , kwargs...)
Beam(            ; kwargs...) = Par(; _constructor = :(Modia3D.Beam)                  , kwargs...)
Cylinder(        ; kwargs...) = Par(; _constructor = :(Modia3D.Cylinder)              , kwargs...)
Sphere(          ; kwargs...) = Par(; _constructor = :(Modia3D.Sphere)                , kwargs...)
Ellipsoid(       ; kwargs...) = Par(; _constructor = :(Modia3D.Ellipsoid)             , kwargs...)
Capsule(         ; kwargs...) = Par(; _constructor = :(Modia3D.Capsule)               , kwargs...)
Cone(            ; kwargs...) = Par(; _constructor = :(Modia3D.Cone)                  , kwargs...)
SpringShape(     ; kwargs...) = Par(; _constructor = :(Modia3D.Spring)                , kwargs...)
GearWheel(       ; kwargs...) = Par(; _constructor = :(Modia3D.GearWheel)             , kwargs...)
Grid(            ; kwargs...) = Par(; _constructor = :(Modia3D.Grid)                  , kwargs...)
VisualMaterial(  ; kwargs...) = Par(; _constructor = :(Modia3D.VisualMaterial)        , kwargs...)
MassProperties(  ; kwargs...) = Par(; _constructor = :(Modia3D.MassProperties)        , kwargs...)
CoordinateSystem(; kwargs...) = Par(; _constructor = :(Modia3D.CoordinateSystem)      , kwargs...)
FileMesh(        ; kwargs...) = Par(; _constructor = :(Modia3D.FileMesh)              , kwargs...)
Font(            ; kwargs...) = Par(; _constructor = :(Modia3D.Font)                  , kwargs...)
TextShape(       ; kwargs...) = Par(; _constructor = :(Modia3D.TextShape)             , kwargs...)
Fix(             ; kwargs...) = Par(; _constructor = :(Modia3D.Fix)                   , kwargs...)

MassPropertiesFromShape()  = Par(; _constructor = :(Modia3D.MassPropertiesFromShape))
MassPropertiesFromShapeAndMass(;mass) = Par(; _constructor = :(Modia3D.MassPropertiesFromShapeAndMass), mass = mass)
UniformGravityField(; kwargs...) = Par(; _constructor = :(Modia3D.UniformGravityField), kwargs...)


RefPath(; kwargs...) = Modia3D.ReferencePath(; kwargs...)

ptpJointSpace(; kwargs...) = Modia3D.ptpJointSpace(; kwargs...)
scheduleReferenceMotion(; kwargs...) = Modia3D.scheduleReferenceMotion(; kwargs...)

calculateRobotMovement(args...) = Modia3D.calculateRobotMovement(args...)
getRefPathPosition(args...) = Modia3D.getRefPathPosition(args...)
getRefPathInitPosition(args...) = Modia3D.getRefPathInitPosition(args...)

getVariables(args...) = (args...,)

multibodyResiduals!(args...)     = Modia3D.multibodyResiduals!(args...)
setModiaJointVariables!(args...) = Modia3D.setModiaJointVariables!(args...)

Revolute(; obj1, obj2, axis=3, phi=Var(init=0.0), w=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Revolute), _path = true, ndof = 1),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    axis = Par(value = axis),
    canCollide = Par(value = canCollide),
    phi  = phi,
    w    = w,
    equations = :[
        w   = der(phi)
        qdd = der(w)   # standardized name for the generalized joint accelerations
        variables = getVariables(phi, w, 0.0) # standardized name for the generalized joint position, velocity, force
        ]
)

RevoluteWithFlange(; obj1, obj2, axis=3, phi=Var(init=0.0), w=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Revolute), _path = true, ndof = 1),
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
        qdd = der(w)   # standardized name for the generalized joint accelerations
        variables = getVariables(phi, w, flange.tau) # standardized name for the generalized joint position, velocity, force
        ]
)

Prismatic(; obj1, obj2, axis=1, s=Var(init=0.0), v=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Prismatic), _path = true, ndof = 1),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    axis = Par(value = axis),
    canCollide = Par(value = canCollide),
    s    = s,
    v    = v,
    equations = :[
        v   = der(s)
        qdd = der(v)   # standardized name for the generalized joint accelerations
        variables = getVariables(s, v, 0.0) # standardized name for the generalized joint position, velocity, force
        ]
)

PrismaticWithFlange(; obj1, obj2, axis=1, s=Var(init=0.0), v=Var(init=0.0), canCollide=true) = Model(; _constructor = Par(value = :(Modia3D.Prismatic), _path = true, ndof = 1),
    obj1   = Par(value = obj1),
    obj2   = Par(value = obj2),
    axis   = Par(value = axis),
    canCollide = Par(value = canCollide),
    flange = TranslationalFlange,
    s      = s,
    v      = v,
    equations = :[
        s   = flange.s
        v   = der(s)
        qdd = der(v)   # standardized name for the generalized joint accelerations
        variables = getVariables(s, v, flange.f) # standardized name for the generalized joint position, velocity, force
        ]
)

"""
    J = J123(rot123::AbstractVector)

Return joint rot. kinematics matrix `J` for Cardan angles `rot123` (rotation sequence x-y-z).
"""
function J123(rot123::AbstractVector)
    #if abs(rot123[2]-pi/2) < 1e-7
    #    error("rot[2] = ", rot123[2], ", that is close to a singularity, leading to a division by zero.")
    #end

    (sbe, cbe) = sincos(rot123[2])
    (sga, cga) = sincos(rot123[3])
    return [     cga     -sga  0.0 ;
             cbe*sga  cbe*cga  0.0 ;
            -sbe*cga  sbe*sga  cbe ] / cbe

end

"""
    J = J132(rot132::AbstractVector)

Return joint rot. kinematics matrix `J` for Cardan angles `rot132` (rotation sequence x-z-y).
"""
function J132(rot132::AbstractVector)

    (sga, cga) = sincos(rot132[2])
    (sbe, cbe) = sincos(rot132[3])
    return [ cbe      0.0  sbe     ;
            -sbe*cga  0.0  cbe*cga ;
             cbe*sga  cga  sbe*sga ] / cga

end



"""
    isrot123 = change_rotSequence!(instantiatedModel, rot, pre_isrot123)
    
Change rotation sequence of `rot` from `x-axis, y-axis, z-axis` to `x-axis, z-axis, y-axis` or visa versa:

- If `pre_isrot123 = true`, return `isrot123 = false` and `rot = rot132fromR(Rfromrot123(rot))`

- If `pre_isrot123 = false`, return `isrot123 = true` and `rot = rot123fromR(Rfromrot132(rot))` 
"""
function change_rotSequence!(rot::AbstractVector, pre_isrot123::Bool, instantiatedModel::SimulationModel, x, rot_name)::Bool
    if pre_isrot123
        rot      = Modia3D.rot132fromR(Modia3D.Rfromrot123(rot)) 
        isrot123 = false
    else
        rot      = Modia3D.rot123fromR(Modia3D.Rfromrot132(rot))
        isrot123 = true
    end
    
    #println("... isrot123 changed to ", isrot123)
    # Change x-vector with the new rot-values
    eqInfo = instantiatedModel.equationInfo
    startIndex = eqInfo.x_info[ eqInfo.x_dict[rot_name] ].startIndex
    x[startIndex]   = rot[1]
    x[startIndex+1] = rot[2]
    x[startIndex+2] = rot[3]
    return isrot123
end


J123or132(rot, isrot123) = isrot123 ? J123(rot) : J132(rot)


FreeMotion(; obj1, obj2, r=Var(init=zeros(3)), rot=Var(start=zeros(3)), v=Var(init=zeros(3)), w=Var(init=zeros(3))) = Model(; _constructor = Par(value = :(Modia3D.FreeMotion), _path = true, ndof = 6),
    obj1 = Par(value = obj1),
    obj2 = Par(value = obj2),
    r    = r,
    rot  = rot,
    v    = v,
    w    = w,
    
    isrot123 = Var(start=true), 
    _rotName = "dummy",  # is changed by buildModia3D to the full path name of "rot"

    equations = :[
        der(r) = v
        
        z_rot123    = abs(rot[2]-1.57) - 0.1
        z_rot123Pos = positive( z_rot123 )
        pre_isrot123 = pre(isrot123)
        isrot123 = if z_rot123Pos; pre_isrot123 else change_rotSequence!(rot, pre_isrot123, instantiatedModel, _x, _rotName) end
        der(rot) = J123or132(rot,isrot123) * w

        der(v) = qdd[1:3]
        der(w) = qdd[4:6]
        variables = getVariables(r, rot, v, w)
        ]
)
