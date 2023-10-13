
function name2uuid(name::String)
    u4 = UUIDs.UUID("07f2c1e0-90b0-56cf-bda7-b44b56e34eed")  # Modia3D package UUID
    return string(UUIDs.uuid5(u4, name))
end

colorNum(red, green, blue) = ((red*256 + green)*256 + blue)

FVal(val) = Modia3D.convertToFloat64(val)

const coSysMaterialRed   = (; name="coordinateSystem.red", uuid=name2uuid("coordinateSystem.red"), type="MeshPhongMaterial", color=colorNum(255, 0, 0), opacity=1, transparent=false, shininess=0.5)
const coSysMaterialGreen = (; name="coordinateSystem.green", uuid=name2uuid("coordinateSystem.green"), type="MeshPhongMaterial", color=colorNum(0, 255, 0), opacity=1, transparent=false, shininess=0.5)
const coSysMaterialBlue  = (; name="coordinateSystem.blue", uuid=name2uuid("coordinateSystem.blue"), type="MeshPhongMaterial", color=colorNum(0, 0, 255), opacity=1, transparent=false, shininess=0.5)


function exportObject(object, elements, obj::Modia3D.Composition.Object3D, sphere::Modia3D.Shapes.Sphere, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Modia3D.NullRotation(Float64)
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="SphereGeometry", radius=0.5*FVal(sphere.diameter), heightSegments=16, widthSegments=32)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot)
    printInfoToFile(object, elements, geometry, material, nothing, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, ellipsoid::Modia3D.Shapes.Ellipsoid, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Modia3D.NullRotation(Float64)
    lengthX = FVal(ellipsoid.lengthX)
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="SphereGeometry", radius=0.5*lengthX, heightSegments=16, widthSegments=32)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, scale=[1.0, FVal(ellipsoid.lengthY)/lengthX, FVal(ellipsoid.lengthZ)/lengthX])
    printInfoToFile(object, elements, geometry, material, nothing, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, box::Modia3D.Shapes.Box, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Modia3D.NullRotation(Float64)
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="BoxGeometry", width=FVal(box.lengthX), height=FVal(box.lengthY), depth=FVal(box.lengthZ))
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot)
    printInfoToFile(object, elements, geometry, material, nothing, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, cylinder::Modia3D.Shapes.Cylinder, initPos, initRot)
    radius = 0.5*FVal(cylinder.diameter)
    innerRadius = 0.5*FVal(cylinder.innerDiameter)
    length = FVal(cylinder.length)
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    if innerRadius == 0.0
        r_obj = Modia3D.ZeroVector3D(Float64)
        R_obj = Shapes.rotateAxis2y(cylinder.axis, Modia3D.NullRotation(Float64))
        shape = nothing
        geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="CylinderGeometry", radiusBottom=radius, radiusTop=radius, height=length, radialSegments=32, heightSegments=1)
        objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, R_obj=R_obj)
    else
        if cylinder.axis == 1
            r_obj = @SVector[-0.5*length, 0.0, 0.0]
        elseif cylinder.axis == 2
            r_obj = @SVector[0.0, -0.5*length, 0.0]
        else
            r_obj = @SVector[0.0, 0.0, -0.5*length]
        end
        R_obj = Shapes.rotateAxis2z(cylinder.axis, Modia3D.NullRotation(Float64))
        innerCurves = [(; type="EllipseCurve", aX=0, aY=0, xRadius=innerRadius, yRadius=innerRadius, aStartAngle=0, aEndAngle=2*pi, aClockwise=false, aRotation=0)]
        holes = [(; type="Path", curves=innerCurves, currentPoint=[0, 0])]
        curves = [(; type="EllipseCurve", aX=0, aY=0, xRadius=radius, yRadius=radius, aStartAngle=0, aEndAngle=2*pi, aClockwise=false, aRotation=0)]
        shapeName = String(Modia3D.fullName(obj)) * ".shape"
        shapeUuid = name2uuid(shapeName)
        shape = (; name=shapeName, uuid=shapeUuid, type="Shape", curves=curves, holes=holes, currentPoint=[0, 0])
        options = (; depth=length, bevelEnabled=false)
        geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="ExtrudeGeometry", shapes=[shapeUuid], options=options)
        objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, r_obj=r_obj, R_obj=R_obj)
    end
    printInfoToFile(object, elements, geometry, material, shape, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, cone::Modia3D.Shapes.Cone, initPos, initRot)
    length = FVal(cone.length)
    if cone.axis == 1
        r_obj = @SVector[0.5*length, 0.0, 0.0]
    elseif cone.axis == 2
        r_obj = @SVector[0.0, 0.5*length, 0.0]
    else
        r_obj = @SVector[0.0, 0.0, 0.5*length]
    end
    R_obj = Shapes.rotateAxis2y(cone.axis, Modia3D.NullRotation(Float64))
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="CylinderGeometry", radiusBottom=0.5*FVal(cone.diameter), radiusTop=0.5*FVal(cone.topDiameter), height=length, radialSegments=32, heightSegments=1)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, r_obj=r_obj, R_obj=R_obj)
    printInfoToFile(object, elements, geometry, material, nothing, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, capsule::Modia3D.Shapes.Capsule, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Shapes.rotateAxis2y(capsule.axis, Modia3D.NullRotation(Float64))
    radius = 0.5*FVal(capsule.diameter)
    length = FVal(capsule.length)
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    points = []
    for i in -9:0
        angle = i/9 * pi/2
        point = (; x=radius*cos(angle), y=-0.5*length+radius*sin(angle))
        push!(points, point)
    end
    for i in 0:9
        angle = i/9 * pi/2
        point = (; x=radius*cos(angle), y=0.5*length+radius*sin(angle))
        push!(points, point)
    end
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="LatheGeometry", points=points, phiStart=0, phiLength=2*pi, segments=32)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, R_obj=R_obj)
    printInfoToFile(object, elements, geometry, material, nothing, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, beam::Modia3D.Shapes.Beam, initPos, initRot)
    length = FVal(beam.length)
    width = FVal(beam.width)
    thickness = FVal(beam.thickness)
    if beam.axis == 1
        r_obj = @SVector[0.0, 0.0, -0.5*thickness]
    elseif beam.axis == 2
        r_obj = @SVector[-0.5*thickness, 0.0, 0.0]
    else
        r_obj = @SVector[0.0, -0.5*thickness, 0.0]
    end
    R_obj = Shapes.rotateAxis2x(beam.axis, Modia3D.NullRotation(Float64))
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    curves = [
        (; type="LineCurve", v1=[-0.5*length, -0.5*width], v2=[0.5*length, -0.5*width]),
        (; type="EllipseCurve", aX=0.5*length, aY=0, xRadius=0.5*width, yRadius=0.5*width, aStartAngle=-pi/2, aEndAngle=pi/2, aClockwise=false, aRotation=0),
        (; type="LineCurve", v1=[0.5*length, 0.5*width], v2=[-0.5*length, 0.5*width]),
        (; type="EllipseCurve", aX=-0.5*length, aY=0, xRadius=0.5*width, yRadius=0.5*width, aStartAngle=pi/2, aEndAngle=-pi/2, aClockwise=false, aRotation=0)
    ]
    shapeName = name * ".shape"
    shapeUuid = name2uuid(shapeName)
    shape = (; name=shapeName, uuid=shapeUuid, type="Shape", curves=curves, holes=[], currentPoint=[0, 0])
    options = (; depth=thickness, bevelEnabled=false)
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="ExtrudeGeometry", shapes=[shapeUuid], options=options)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, r_obj=r_obj, R_obj=R_obj)
    printInfoToFile(object, elements, geometry, material, shape, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, coordinateSystem::Modia3D.Shapes.CoordinateSystem, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Modia3D.NullRotation(Float64)
    name = Modia3D.fullName(obj)
    group = (; name=name, uuid=name2uuid(name), type="Group", children=[], position=initPos, rotation=Modia3D.rot123fromR(initRot), scale=ones(3))

    geometryName = name * ".axis.geometry"
    geometryUuid = name2uuid(geometryName)
    geometry = (; name=geometryName, uuid=geometryUuid, type="CylinderGeometry", radiusBottom=coordinateSystem.length/50, radiusTop=coordinateSystem.length/50, height=coordinateSystem.length, radialSegments=8, heightSegments=1)
    push!(elements.geometries, geometry)

    r_geo = @SVector[coordinateSystem.length/2, 0.0, 0.0]
    R_geo = Shapes.rotateAxis2y(1, Modia3D.NullRotation(Float64))
    child = (; type="Mesh", geometry=geometryUuid, material=coSysMaterialRed.uuid, position=r_geo, rotation=Modia3D.rot123fromR(R_geo), scale=ones(3))
    push!(group.children, child)

    r_geo = @SVector[0.0, coordinateSystem.length/2, 0.0]
    R_geo = Shapes.rotateAxis2y(2, Modia3D.NullRotation(Float64))
    child = (; type="Mesh", geometry=geometryUuid, material=coSysMaterialGreen.uuid, position=r_geo, rotation=Modia3D.rot123fromR(R_geo), scale=ones(3))
    push!(group.children, child)

    r_geo = @SVector[0.0, 0.0, coordinateSystem.length/2]
    R_geo = Shapes.rotateAxis2y(3, Modia3D.NullRotation(Float64))
    child = (; type="Mesh", geometry=geometryUuid, material=coSysMaterialBlue.uuid, position=r_geo, rotation=Modia3D.rot123fromR(R_geo), scale=ones(3))
    push!(group.children, child)

    geometryName = name * ".tip.geometry"
    geometryUuid = name2uuid(geometryName)
    geometry = (; name=geometryName, uuid=geometryUuid, type="CylinderGeometry", radiusBottom=coordinateSystem.length/25, radiusTop=0.0, height=coordinateSystem.length/10, radialSegments=8, heightSegments=1)
    push!(elements.geometries, geometry)

    r_geo = @SVector[1.05*coordinateSystem.length, 0.0, 0.0]
    R_geo = Shapes.rotateAxis2y(1, Modia3D.NullRotation(Float64))
    child = (; type="Mesh", geometry=geometryUuid, material=coSysMaterialRed.uuid, position=r_geo, rotation=Modia3D.rot123fromR(R_geo), scale=ones(3))
    push!(group.children, child)

    r_geo = @SVector[0.0, 1.05*coordinateSystem.length, 0.0]
    R_geo = Shapes.rotateAxis2y(2, Modia3D.NullRotation(Float64))
    child = (; type="Mesh", geometry=geometryUuid, material=coSysMaterialGreen.uuid, position=r_geo, rotation=Modia3D.rot123fromR(R_geo), scale=ones(3))
    push!(group.children, child)

    r_geo = @SVector[0.0, 0.0, 1.05*coordinateSystem.length]
    R_geo = Shapes.rotateAxis2y(3, Modia3D.NullRotation(Float64))
    child = (; type="Mesh", geometry=geometryUuid, material=coSysMaterialBlue.uuid, position=r_geo, rotation=Modia3D.rot123fromR(R_geo), scale=ones(3))
    push!(group.children, child)

    push!(object.children, group)
    return (r_obj, R_obj)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, grid::Modia3D.Shapes.Grid, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Shapes.rotateAxis2z(grid.axis, Modia3D.NullRotation(Float64))
    name = Modia3D.fullName(obj)
    geometryName = name * ".geometry"
    array = []
    for i in 0:Int(div(grid.length+1e-12, grid.distance))
        x = i * grid.distance - grid.length/2
        push!(array, x, -grid.width/2, 0)
        push!(array, x,  grid.width/2, 0)
    end
    for i in 0:Int(div(grid.width+1e-12, grid.distance))
        y = i * grid.distance - grid.width/2
        push!(array, -grid.length/2, y, 0)
        push!(array,  grid.length/2, y, 0)
    end
    position = (; itemSize=3, type="Float32Array", array=array)
    geometry = (; name=geometryName, uuid=name2uuid(geometryName), type="BufferGeometry", data=(; attributes=(; position=position)))
    materialName = name * ".material"
    material = (; name=materialName, uuid=name2uuid(materialName), type="LineBasicMaterial", color=colorNum(0, 0, 255), linewidth=grid.lineWidth)
    objectInfo = getObjectInfo(name, geometry, material, initPos, initRot, type="LineSegments", R_obj=R_obj)
    printInfoToFile(object, elements, geometry, material, nothing, objectInfo)
    return (r_obj, R_obj)
end

function printInfoToFile(object, elements, geometry, material, shape, objectInfo)
    push!(elements.geometries, geometry)
    if !isnothing(material)
        push!(elements.materials, material)
    end
    if !isnothing(shape)
        push!(elements.shapes, shape)
    end
    push!(object.children, objectInfo)
end

function exportObject(object, elements, obj::Modia3D.Composition.Object3D, fileMesh::Modia3D.Shapes.FileMesh, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Modia3D.NullRotation(Float64)
    filenameOld = fileMesh.filename
    filename = filenameOld
    (head,ext) = splitext(filenameOld)
    if ext == ".json"
        filename = filenameOld
    elseif ext == ".obj"
        filename = head * ".json"
        if !isfile(filename)
            @warn("Please convert $filenameOld into a .json file.")
            return nothing
        end
    else
        @warn("Please convert $filenameOld into a .json file.")
        return nothing
    end

    io = open(filename, "r")
    meshObject = JSON.parse(io)
    close(io)

    if fileMesh.useMaterialColor || fileMesh.doubleSided
        # overwrite materials of mesh object children
        materialName = Modia3D.fullName(obj) * ".material"
        for material in meshObject["materials"]
            push!(material, "name" => materialName, "uuid" => name2uuid(materialName))
            if fileMesh.useMaterialColor
                push!(material, "color" => colorNum(obj.visualMaterial.color[1], obj.visualMaterial.color[2], obj.visualMaterial.color[3]))
            end
            if fileMesh.doubleSided
                push!(material, "side" => 2)  # THREE.DoubleSide
            end
        end
        for child in (meshObject["object"])["children"]
            push!(child, "material" => name2uuid(materialName))
        end
    end

    push!(elements.geometries, meshObject["geometries"]...)
    push!(elements.materials, meshObject["materials"]...)
    objectInfo = getObjectInfoMesh(obj, initPos, initRot, fileMesh.scaleFactor, R_obj, meshObject["object"])
    push!(object.children, objectInfo)

    return (r_obj, R_obj)
end

function getObjectInfoMesh(obj, initPos, initRot, scale, R_obj, meshInfo)
    push!(meshInfo, "name" => Modia3D.fullName(obj) )
    delete!(meshInfo, "matrix")
    push!(meshInfo, "position" => initPos )
    push!(meshInfo, "rotation" => Modia3D.rot123fromR(R_obj*initRot) )
    push!(meshInfo, "scale" => scale )

    return meshInfo
end

function getObjectInfo(name, geometry, material, initPos, initRot; type="Mesh", r_obj=Modia3D.ZeroVector3D(Float64), R_obj=Modia3D.NullRotation(Float64), scale=ones(3))
    return (; name=name, uuid=name2uuid(name), type=type, geometry=get(geometry, :uuid, nothing), material=get(material, :uuid, ""), position=initPos+initRot'*r_obj, rotation=Modia3D.rot123fromR(R_obj*initRot), scale=scale)
end

printVisuMaterialToJSON(obj, visuMaterial) = nothing
function printVisuMaterialToJSON(obj, visuMaterial::Modia3D.Shapes.VisualMaterial)
    name = String(Modia3D.fullName(obj)) * ".material"

    transparent = visuMaterial.transparency > 0

    material = (; name=name, uuid=name2uuid(name), type="MeshPhongMaterial", color=colorNum(visuMaterial.color[1], visuMaterial.color[2], visuMaterial.color[3]), opacity=1-visuMaterial.transparency, transparent=transparent,
    shininess=visuMaterial.shininess*100)

    return material
end

function printObjectToJSON(object, elements, obj; initPos=obj.r_abs, initRot=obj.R_abs)
    r_obj = Modia3D.ZeroVector3D(Float64)
    R_obj = Modia3D.NullRotation(Float64)
    shapeKind = obj.shapeKind

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Shapes.Sphere = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, sphere, initPos, initRot)

    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Shapes.Ellipsoid = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, ellipsoid, initPos, initRot)

    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Shapes.Box = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, box, initPos, initRot)

    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Shapes.Cylinder = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, cylinder, initPos, initRot)

    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Shapes.Cone = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, cone, initPos, initRot)

    elseif shapeKind == Modia3D.CapsuleKind
        capsule::Modia3D.Shapes.Capsule = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, capsule, initPos, initRot)

    elseif shapeKind == Modia3D.BeamKind
        beam::Modia3D.Shapes.Beam = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, beam, initPos, initRot)

    elseif shapeKind == Modia3D.CoordinateSystemKind
        coordinateSystem::Modia3D.Shapes.CoordinateSystem = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, coordinateSystem, initPos, initRot)

    elseif shapeKind == Modia3D.GridKind
        grid::Modia3D.Shapes.Grid = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, grid, initPos, initRot)

    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.Shapes.FileMesh = obj.shape
        (r_obj, R_obj) = exportObject(object, elements, obj, fileMesh, initPos, initRot)

    else
        @warn("$shapeKind of $(Modia3D.fullName(obj)) is not supported by animation export.")
        return (nothing, nothing)
    end
    return (r_obj, R_obj)
end

function addLights!(object, uuid4, position)
    name = "Directional Light"
    uuid = string(UUIDs.uuid5(UUIDs.UUID(uuid4), name))
    directionalLight = (; name=name, uuid=uuid, type="DirectionalLight", position=position)
    push!(object.children, directionalLight)

    name="Ambient Light"
    uuid = string(UUIDs.uuid5(UUIDs.UUID(uuid4), name))
    ambientLight = (; name=name, uuid=uuid, type="AmbientLight", intensity=0.5)
    push!(object.children, ambientLight)
end

function addCameras!(object, uuid4, position, orientation)
    radius = LinearAlgebra.norm(position)

    name = "Perspective Camera"
    uuid = string(UUIDs.uuid5(UUIDs.UUID(uuid4), name))
    camera = (; name=name, uuid=uuid, type="PerspectiveCamera", position=position, rotation=Modia3D.rot123fromR(orientation))
    push!(object.children, camera)

    name="Orthographic Camera XY"
    uuid = string(UUIDs.uuid5(UUIDs.UUID(uuid4), name))
    camera = (; name=name, uuid=uuid, type="OrthographicCamera", position=[0, 0, radius], rotation=[0, 0, 0], bottom=-radius/2, top=radius/2, left=-radius/2, right=radius/2)
    push!(object.children, camera)

    name="Orthographic Camera YZ"
    uuid = string(UUIDs.uuid5(UUIDs.UUID(uuid4), name))
    camera = (; name=name, uuid=uuid, type="OrthographicCamera", position=[radius, 0, 0], rotation=[0, deg2rad(90), deg2rad(90)], bottom=-radius/2, top=radius/2, left=-radius/2, right=radius/2)
    push!(object.children, camera)

    name="Orthographic Camera ZX"
    uuid = string(UUIDs.uuid5(UUIDs.UUID(uuid4), name))
    camera = (; name=name, uuid=uuid, type="OrthographicCamera", position=[0, radius, 0], rotation=[deg2rad(-90), 0, deg2rad(-90)], bottom=-radius/2, top=radius/2, left=-radius/2, right=radius/2)
    push!(object.children, camera)
end

function createAnimationPositionTrack(tracks, animation, obj, iobj, r_obj)
    keys = []
    for istep in 1:lastindex(animation)
        r_all = animation[istep].objectData[iobj].position + (Modia3D.from_q(animation[istep].objectData[iobj].quaternion))'*r_obj
        push!(keys, (; time=animation[istep].time, value=r_all) )
    end
    name = string(Modia3D.fullName(obj), ".position")
    push!(tracks, (; name=name, uuid=name2uuid(name), type="vector3", keys ) )
end

function createAnimationQuaternionTrack(tracks, animation, obj, iobj, R_obj)
    keys = []
    for istep in 1:lastindex(animation)
        q_all = Modia3D.absoluteRotation(animation[istep].objectData[iobj].quaternion, Modia3D.from_R(R_obj))
        push!(keys, (; time=animation[istep].time, value=q_all) )
    end
    name = string(Modia3D.fullName(obj), ".quaternion")
    push!(tracks, (; name=name, uuid=name2uuid(name), type="quaternion", keys ) )

end

function createAnimationQuaternionTrack(object, animation, obj, iobj, R_obj::Nothing)
    return nothing
end


function exportAnimation(scene)
    allVisuElements = scene.allVisuElements
    if scene.exportAnimation && length(allVisuElements) > 0
        animationFile = scene.options.animationFile
        (head,ext) = splitext(animationFile)
        if ext != ".json"
            @warn("The path of an animationFile=$animationFile must end with .json.")
            return nothing
        end
        print("Export animation to $animationFile ... ")
        animation = scene.animation
        elements = (; geometries=[], materials=[], shapes=[])
        metadata = (; generator = "Modia3D", type = "Object")
        name = scene.name
        uuid = name2uuid(name)
        object = (; name, uuid, animations=[uuid], children=[])

        options = scene.options
        cameraUpDir = 2
        sceneUpDir = Modia3D.upwardsDirection(options.gravityField)
        (r_Light, R_Light) = Modia3D.cameraPosition(options.lightDistance, options.lightLongitude, options.lightLatitude, cameraUpDir, sceneUpDir)
        addLights!(object, uuid, r_Light)
        (r_Camera, R_Camera) = Modia3D.cameraPosition(options.cameraDistance, options.cameraLongitude, options.cameraLatitude, cameraUpDir, sceneUpDir)
        addCameras!(object, uuid, r_Camera, R_Camera)

        push!(elements.materials, coSysMaterialRed)
        push!(elements.materials, coSysMaterialGreen)
        push!(elements.materials, coSysMaterialBlue)

        if !isnothing(animation) && length(animation) != 0
            iobj = 0
            tracks = []
            for obj in allVisuElements
                iobj = iobj + 1
                (r_obj, R_obj) = printObjectToJSON(object, elements, obj, initPos=animation[1].objectData[iobj].position, initRot=Modia3D.from_q(animation[1].objectData[iobj].quaternion))
                if !isnothing(R_obj)
                    createAnimationPositionTrack(tracks, animation, obj, iobj, r_obj)
                    createAnimationQuaternionTrack(tracks, animation, obj, iobj, R_obj)
                end
            end
            animations = [(; name="Simulation", uuid=uuid, tracks)]
            scene = (; metadata, elements.geometries, elements.materials, elements.shapes, object, animations)
        else
            for obj in allVisuElements
                (r_obj, R_obj) = printObjectToJSON(object, elements, obj)
            end
            scene = (; metadata, elements.geometries, elements.materials, elements.shapes, object)
        end

        io = open(animationFile, "w")
        JSON.print(io, scene, 0)  # 3rd parameter: nothing = not human readable; 0 = no indentation; n = number of indentation spaces
        close(io)

        println("done.")
    end
end


function Composition.isVisible(feature::Shapes.Solid{F}, exportAnimation::Bool) where F <: Modia3D.VarFloatType
    return exportAnimation && !isnothing(feature.shape) && !isnothing(feature.visualMaterial)
end

function Composition.isVisible(feature::Shapes.Visual, exportAnimation::Bool)
    return exportAnimation && !isnothing(feature.visualMaterial) && !isnothing(feature.shape)
end
