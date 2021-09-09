# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

@enum ShapeType SimVisBox=1 SimVisSphere=2 SimVisCylinder=3 SimVisCone=4 SimVisCapsule=5 SimVisCoordSys=6 SimVisSpring=7 SimVisGearWheel=8 SimVisPipe=9 SimVisGrid=10 SimVisBeam=11

const mvecSize = MVector{3,Cdouble}(0.0, 0.0, 0.0)
const DefaultMaterial = Shapes.VisualMaterial()
const emptyShaderName=" "


@inline function visualizeShape(simVis::SimVis_Renderer,
                                rabs::SVector{3,Float64},
                                Rabs::SMatrix{3,3,Float64,9},
                                shape::ShapeType,
                                id::Ptr{Nothing},
                                material::Shapes.VisualMaterial,
                                Lx::Float64, Ly::Float64, Lz::Float64;
                                extras::MVector{3,Float64}=Basics.ZeroMVector)
    mvecSize[1] = Lx
    mvecSize[2] = Ly
    mvecSize[3] = Lz

    SimVis_setBaseObject(simVis, id, Cint(0), Cint(shape), rabs, Rabs, mvecSize,
        material.color, Cint(material.wireframe), Cint(material.reflectslight),
        material.shininess, extras, material.transparency, Cint(0), Cint(material.shadowMask))
end


function visualizeObject(obj::Composition.Object3D, id::Ptr{Nothing}, simVis::SimVis_Renderer)
    shapeKind = obj.shapeKind

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Sphere = obj.shape
        visualizeShape(simVis, obj.r_abs, obj.R_abs, SimVisSphere, id, obj.visualMaterial,
                       sphere.diameter, sphere.diameter, sphere.diameter)

    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Ellipsoid = obj.shape
        visualizeShape(simVis, obj.r_abs, obj.R_abs, SimVisSphere, id, obj.visualMaterial,
                       ellipsoid.lengthX, ellipsoid.lengthY, ellipsoid.lengthZ)

    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Box = obj.shape
        visualizeShape(simVis, obj.r_abs, obj.R_abs, SimVisBox, id, obj.visualMaterial,
                       box.lengthX, box.lengthY, box.lengthZ)

    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Cylinder = obj.shape
        if cylinder.innerDiameter == 0.0
            visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2z(cylinder.axis, obj.R_abs), SimVisCylinder, id, obj.visualMaterial,
                           cylinder.diameter, cylinder.diameter, cylinder.length)
        else
            visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2z(cylinder.axis, obj.R_abs), SimVisPipe, id, obj.visualMaterial,
                           cylinder.diameter, cylinder.diameter, cylinder.length; extras=@MVector[0.0, cylinder.innerDiameter/cylinder.diameter, 1.0])
        end

    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Cone = obj.shape
        if cone.axis == 1
            dr = @MVector[cone.length/4, 0.0, 0.0]
        elseif cone.axis == 2
            dr = @MVector[0.0, cone.length/4, 0.0]
        else
            dr = @MVector[0.0, 0.0, cone.length/4]
        end
        r_abs = obj.r_abs + obj.R_abs' * dr
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(cone.axis, obj.R_abs), SimVisCone, id, obj.visualMaterial,
                       cone.diameter, cone.diameter, cone.length; extras=@MVector[cone.topDiameter/cone.diameter, 0.0, 0.0])

    elseif shapeKind == Modia3D.CapsuleKind
        capsule::Modia3D.Capsule = obj.shape
        visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2z(capsule.axis, obj.R_abs), SimVisCapsule, id, obj.visualMaterial,
                       capsule.diameter, capsule.diameter, capsule.length)

    elseif shapeKind == Modia3D.BeamKind
        beam::Modia3D.Beam = obj.shape
        visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2x(beam.axis, obj.R_abs), SimVisBeam, id, obj.visualMaterial,
                       beam.length, beam.width, beam.thickness)

    elseif shapeKind == Modia3D.CoordinateSystemKind
        coordinateSystem::Modia3D.CoordinateSystem = obj.shape
        visualizeShape(simVis, obj.r_abs, obj.R_abs, SimVisCoordSys, id, DefaultMaterial,
                       coordinateSystem.length, coordinateSystem.length, coordinateSystem.length)

    elseif shapeKind == Modia3D.GridKind
        grid::Modia3D.Grid = obj.shape
        visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2z(grid.axis, obj.R_abs), SimVisGrid, id, DefaultMaterial,
                       grid.length, grid.width, grid.length; extras=@MVector[grid.distance, grid.lineWidth, 0.0])

    elseif shapeKind == Modia3D.SpringKind
        spring::Modia3D.Spring = obj.shape
        visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2z(spring.axis, obj.R_abs), SimVisSpring, id, obj.visualMaterial,
                       spring.length, spring.diameter, spring.diameter; extras=@MVector[spring.windings, spring.wireDiameter/2, 0.0])

    elseif shapeKind == Modia3D.GearWheelKind
        gearWheel::Modia3D.GearWheel = obj.shape
        visualizeShape(simVis, obj.r_abs, Shapes.rotateAxis2z(gearWheel.axis, obj.R_abs), SimVisGearWheel, id, obj.visualMaterial,
                       gearWheel.diameter, gearWheel.diameter, gearWheel.length; extras=@MVector[gearWheel.innerDiameter/gearWheel.diameter, gearWheel.teeth, gearWheel.angle*180/pi])

    elseif shapeKind == Modia3D.ModelicaKind
        modelica::Modia3D.ModelicaShape = obj.shape
        visualizeShape(simVis, obj.r_abs, obj.R_abs, ShapeType(modelica.type), id, obj.visualMaterial,
                       modelica.lengthX, modelica.lengthY, modelica.lengthZ; extras=modelica.extra)

    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.FileMesh = obj.shape
        SimVis_setFileObject(simVis, id, Cint(0), obj.r_abs, obj.R_abs,
                             MVector{3,Float64}(fileMesh.scaleFactor), Cint(obj.visualMaterial.reflectslight), obj.visualMaterial.shininess, obj.visualMaterial.transparency, Cint(obj.visualMaterial.wireframe), Cint(0),
                             fileMesh.filename, Cint(fileMesh.smoothNormals), fileMesh.useMaterialColor, MVector{3,Cint}(obj.visualMaterial.color),
                             Cint(obj.visualMaterial.shadowMask), emptyShaderName)

    elseif shapeKind == Modia3D.TextKind
        textShape::Modia3D.TextShape = obj.shape
        SimVis_setTextObject(simVis, id, Cint(textShape.axisAlignment), textShape.text, 0.0, Cint(0), obj.r_abs, obj.R_abs,
                             textShape.font.charSize, textShape.font.fontFileName, MVector{3,Cint}(textShape.font.color), textShape.font.transparency,
                             textShape.offset, Cint(textShape.alignment), Cint(0))

    end
end
