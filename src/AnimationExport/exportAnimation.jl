
function name2uuid(name::String)
    u4 = UUIDs.UUID("07f2c1e0-90b0-56cf-bda7-b44b56e34eed")  # Modia3D package UUID
    uuid = string(UUIDs.uuid5(u4, name))
end


function exportObject(object, shapes, obj::Modia3D.Composition.Object3D, sphere::Modia3D.Shapes.Sphere, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D
    R_obj = Modia3D.NullRotation
    name = String(Modia3D.fullName(obj)) * ".geometry"
    shape = (; name=name, uuid=name2uuid(name), type="SphereBufferGeometry", radius=sphere.diameter/2, heightSegments=16, widthSegments=32)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(obj, shape, material, initPos, initRot)
    printInfoToFile(object, shapes, shape, material, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, shapes, obj::Modia3D.Composition.Object3D, ellipsoid::Modia3D.Shapes.Ellipsoid, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D
    R_obj = Modia3D.NullRotation
    name = String(Modia3D.fullName(obj)) * ".geometry"
    shape = (; name=name, uuid=name2uuid(name), type="SphereBufferGeometry", radius=ellipsoid.lengthX/2, heightSegments=16, widthSegments=32)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(obj, shape, material, initPos, initRot, scale=[1.0, ellipsoid.lengthY/ellipsoid.lengthX, ellipsoid.lengthZ/ellipsoid.lengthX])
    printInfoToFile(object, shapes, shape, material, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, shapes, obj::Modia3D.Composition.Object3D, box::Modia3D.Shapes.Box, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D
    R_obj = Modia3D.NullRotation
    name = String(Modia3D.fullName(obj)) * ".geometry"
    shape = (; name=name, uuid=name2uuid(name), type="BoxBufferGeometry", width=box.lengthX, height=box.lengthY, depth=box.lengthZ)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(obj, shape, material, initPos, initRot)
    printInfoToFile(object, shapes, shape, material, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, shapes, obj::Modia3D.Composition.Object3D, cylinder::Modia3D.Shapes.Cylinder, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D
    if cylinder.axis == 1
        R_obj = @SMatrix[0 -1 0; 1 0 0; 0 0 1]
    elseif cylinder.axis == 2
        R_obj = Modia3D.NullRotation
    else
        R_obj = @SMatrix[1 0 0; 0 0 1; 0 -1 0]
    end
    name = String(Modia3D.fullName(obj)) * ".geometry"
    shape = (; name=name, uuid=name2uuid(name), type="CylinderBufferGeometry", radiusBottom=cylinder.diameter/2, radiusTop=cylinder.diameter/2, height=cylinder.length, radialSegments=32, heightSegments=1)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(obj, shape, material, initPos, initRot, R_obj=R_obj)
    printInfoToFile(object, shapes, shape, material, objectInfo)
    return (r_obj, R_obj)
end

function exportObject(object, shapes, obj::Modia3D.Composition.Object3D, cone::Modia3D.Shapes.Cone, initPos, initRot)
    if cone.axis == 1
        r_obj = @SVector[cone.length/2, 0.0, 0.0]
        R_obj = @SMatrix[0 -1 0; 1 0 0; 0 0 1]
    elseif cone.axis == 2
        r_obj = @SVector[0.0, cone.length/2, 0.0]
        R_obj = Modia3D.NullRotation
    else
        r_obj = @SVector[0.0, 0.0, cone.length/2]
        R_obj = @SMatrix[1 0 0; 0 0 1; 0 -1 0]
    end
    name = String(Modia3D.fullName(obj)) * ".geometry"
    shape = (; name=name, uuid=name2uuid(name), type="CylinderBufferGeometry", radiusBottom=cone.diameter/2, radiusTop=cone.topDiameter/2, height=cone.length, radialSegments=32, heightSegments=1)
    material = printVisuMaterialToJSON(obj, obj.visualMaterial)
    objectInfo = getObjectInfo(obj, shape, material, initPos, initRot, r_obj=r_obj, R_obj=R_obj)
    printInfoToFile(object, shapes, shape, material, objectInfo)
    return (r_obj, R_obj)
end

function printInfoToFile(object, shapes, shape, material, objectInfo)
    push!(shapes.geometries, shape)
    push!(shapes.materials, material)
    push!(object.children, objectInfo)
end

function exportObject(object, shapes, obj::Modia3D.Composition.Object3D, fileMesh::Modia3D.Shapes.FileMesh, initPos, initRot)
    r_obj = Modia3D.ZeroVector3D
    R_obj = Modia3D.NullRotation
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

    push!(shapes.geometries, meshObject["geometries"]...)
    push!(shapes.materials, meshObject["materials"]...)
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

function getObjectInfo(obj, shape, material, initPos, initRot; r_obj=Modia3D.ZeroVector3D, R_obj=Modia3D.NullRotation, scale=ones(3))
    name = String(Modia3D.fullName(obj))
    return (; name=name, uuid=name2uuid(name), type=:Mesh, geometry=get(shape, :uuid, nothing), material=get(material, :uuid, ""), position=initPos+initRot'*r_obj, rotation=Modia3D.rot123fromR(R_obj*initRot), scale=scale )
end

printVisuMaterialToJSON(obj, visuMaterial) = nothing
function printVisuMaterialToJSON(obj, visuMaterial::Modia3D.Shapes.VisualMaterial)
    name = String(Modia3D.fullName(obj)) * ".material"

    transparent = visuMaterial.transparency > 0

    material = (; name=name, uuid=name2uuid(name), type="MeshPhongMaterial", color=((visuMaterial.color[1]*256 + visuMaterial.color[2])*256 + visuMaterial.color[3]), opacity=1-visuMaterial.transparency, transparent=transparent,
    shininess=visuMaterial.shininess*100, side="DoubleSide")

    return material
end

function printObjectToJSON(object, shapes, obj; initPos=obj.r_abs, initRot=obj.R_abs)
    r_obj = Modia3D.ZeroVector3D
    R_obj = Modia3D.NullRotation
    shapeKind = obj.shapeKind

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Shapes.Sphere = obj.shape
        (r_obj, R_obj) = exportObject(object, shapes, obj, sphere, initPos, initRot)

    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Shapes.Ellipsoid = obj.shape
        (r_obj, R_obj) = exportObject(object, shapes, obj, ellipsoid, initPos, initRot)

    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Shapes.Box = obj.shape
        (r_obj, R_obj) = exportObject(object, shapes, obj, box, initPos, initRot)

    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Shapes.Cylinder = obj.shape
        (r_obj, R_obj) = exportObject(object, shapes, obj, cylinder, initPos, initRot)

    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Shapes.Cone = obj.shape
        (r_obj, R_obj) = exportObject(object, shapes, obj, cone, initPos, initRot)

    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.Shapes.FileMesh = obj.shape
        (r_obj, R_obj) = exportObject(object, shapes, obj, fileMesh, initPos, initRot)

    else
        @warn("$shapeKind is not supported by animation export.")
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
    camera = (; name=name, uuid=uuid, type="PerspectiveCamera", position=position, rotation=Modia3D.rot123fromR(SMatrix{3,3,Float64}(orientation)))
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
    for istep in 1:length(animation)
        r_all = animation[istep].objectData[iobj].position + (Modia3D.from_q(SVector{4,Float64}(animation[istep].objectData[iobj].quaternion)))'*r_obj
        push!(keys, (; time=animation[istep].time, value=r_all) )
    end
    name = string(Modia3D.fullName(obj), ".position")
    push!(tracks, (; name=name, uuid=name2uuid(name), type="vector3", keys ) )
end

function createAnimationQuaternionTrack(tracks, animation, obj, iobj, R_obj)
    keys = []
    for istep in 1:length(animation)
        R_all = Modia3D.from_R(R_obj*Modia3D.from_q(SVector{4,Float64}(animation[istep].objectData[iobj].quaternion)) )
        push!(keys, (; time=animation[istep].time, value=R_all) )
    end
    name = string(Modia3D.fullName(obj), ".quaternion")
    push!(tracks, (; name=name, uuid=name2uuid(name), type="quaternion", keys ) )

end

function createAnimationQuaternionTrack(object, animation, obj, iobj, R_obj::Nothing)
    return nothing
end


function exportAnimation(scene)
    animationFile = scene.options.animationFile
    allVisuElements = scene.allVisuElements
    animation = scene.animation
    if length(allVisuElements) > 0 && !isnothing(animationFile)
        (head,ext) = splitext(animationFile)
        if ext != ".json"
            @warn("The path of an animationFile=$animationFile must end with .json.")
            return nothing
        end
        print("Export animation to $animationFile ... ")
        shapes = (; geometries=[], materials=[])
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

        if !isnothing(animation) && length(animation) != 0
            iobj = 0
            tracks = []
            for obj in allVisuElements
                iobj = iobj + 1
                (r_obj, R_obj) = printObjectToJSON(object, shapes, obj, initPos=animation[1].objectData[iobj].position, initRot=Modia3D.from_q(SVector{4,Float64}(animation[1].objectData[iobj].quaternion)) )
                if !isnothing(R_obj)
                    createAnimationPositionTrack(tracks, animation, obj, iobj, r_obj)
                    createAnimationQuaternionTrack(tracks, animation, obj, iobj, R_obj)
                end
            end
            animations = [(; name="Simulation", uuid=uuid, tracks)]
            scene = (; metadata, shapes.geometries, shapes.materials, object, animations)
        else
            for obj in allVisuElements
                (r_obj, R_obj) = printObjectToJSON(object, shapes, obj)
            end
            scene = (; metadata, shapes.geometries, shapes.materials, object)
        end

        io = open(animationFile, "w")
        JSON.print(io, scene, 0)  # 3rd parameter: nothing = not human readable; 0 = no indentation; n = number of indentation spaces
        close(io)

        println("done.")
    end
end
