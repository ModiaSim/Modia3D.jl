# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

@enum ShapeType SimVisBox=1 SimVisSphere=2 SimVisCylinder=3 SimVisCone=4 SimVisCapsule=5 SimVisCoordSys=6 SimVisSpring=7 SimVisGearWheel=8 SimVisPipe=9 SimVisGrid=10 SimVisBeam=11

const mvecSize = MVector{3,Cdouble}(0.0, 0.0, 0.0)
const DefaultMaterial = Shapes.VisualMaterial()
const emptyShaderName = " "


# convert VarFloatType to Float64
FVal(val) = Modia3D.convertToFloat64(val)
FVec(vec) = SVector{3,Float64}(Modia3D.convertToFloat64(vec))
FMat(mat) = SMatrix{3,3,Float64,9}(Modia3D.convertToFloat64(mat))


@inline function visualizeShape(simVis::SimVis_Renderer,
                                rabs::SVector{3,Float64},
                                Rabs::SMatrix{3,3,Float64,9},
                                shape::ShapeType,
                                id::Ptr{Nothing},
                                material::Shapes.VisualMaterial,
                                Lx::Float64, Ly::Float64, Lz::Float64;
                                extras::MVector{3,Float64}=@MVector[0.0, 0.0, 0.0])
    mvecSize[1] = Lx
    mvecSize[2] = Ly
    mvecSize[3] = Lz

    SimVis_setBaseObject(simVis, id, Cint(0), Cint(shape), rabs, Rabs, mvecSize,
        material.color, Cint(material.wireframe), Cint(material.reflectslight),
        material.shininess, extras, material.transparency, Cint(0), Cint(material.shadowMask))

    return nothing
end


function visualizeObject(obj::Composition.Object3D{F}, id::Ptr{Nothing}, simVis::SimVis_Renderer) where F <: Modia3D.VarFloatType
    shapeKind = obj.shapeKind
    r_abs = FVec(obj.r_abs)
    R_abs = FMat(obj.R_abs)

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Shapes.Sphere{F} = obj.shape
        visualizeShape(simVis, r_abs, R_abs, SimVisSphere, id, obj.visualMaterial,
                       FVal(sphere.diameter), FVal(sphere.diameter), FVal(sphere.diameter))

    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Shapes.Ellipsoid{F} = obj.shape
        visualizeShape(simVis, r_abs, R_abs, SimVisSphere, id, obj.visualMaterial,
        FVal(ellipsoid.lengthX), FVal(ellipsoid.lengthY), FVal(ellipsoid.lengthZ))

    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Shapes.Box{F} = obj.shape
        visualizeShape(simVis, r_abs, R_abs, SimVisBox, id, obj.visualMaterial,
        FVal(box.lengthX), FVal(box.lengthY), FVal(box.lengthZ))

    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Shapes.Cylinder{F} = obj.shape
        if cylinder.innerDiameter == 0.0
            visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(cylinder.axis, R_abs), SimVisCylinder, id, obj.visualMaterial,
            FVal(cylinder.diameter), FVal(cylinder.diameter), FVal(cylinder.length))
        else
            visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(cylinder.axis, R_abs), SimVisPipe, id, obj.visualMaterial,
            FVal(cylinder.diameter), FVal(cylinder.diameter), FVal(cylinder.length); extras=@MVector[0.0, FVal(cylinder.innerDiameter/cylinder.diameter), 1.0])
        end

    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Shapes.Cone{F} = obj.shape
        if cone.axis == 1
            dr = SVector{3,Float64}([0.25*FVal(cone.length), 0.0, 0.0])
        elseif cone.axis == 2
            dr = SVector{3,Float64}([0.0, 0.25*FVal(cone.length), 0.0])
        else
            dr = SVector{3,Float64}([0.0, 0.0, 0.25*FVal(cone.length)])
        end
        r_abs = r_abs + R_abs' * dr
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(cone.axis, R_abs), SimVisCone, id, obj.visualMaterial,
        FVal(cone.diameter), FVal(cone.diameter), FVal(cone.length); extras=@MVector[FVal(cone.topDiameter/cone.diameter), 0.0, 0.0])

    elseif shapeKind == Modia3D.CapsuleKind
        capsule::Modia3D.Shapes.Capsule{F} = obj.shape
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(capsule.axis, R_abs), SimVisCapsule, id, obj.visualMaterial,
        FVal(capsule.diameter), FVal(capsule.diameter), FVal(capsule.length))

    elseif shapeKind == Modia3D.BeamKind
        beam::Modia3D.Shapes.Beam{F} = obj.shape
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2x(beam.axis, R_abs), SimVisBeam, id, obj.visualMaterial,
        FVal(beam.length), FVal(beam.width), FVal(beam.thickness) )

    elseif shapeKind == Modia3D.CoordinateSystemKind
        coordinateSystem::Modia3D.Shapes.CoordinateSystem = obj.shape
        visualizeShape(simVis, r_abs, R_abs, SimVisCoordSys, id, DefaultMaterial,
                       coordinateSystem.length, coordinateSystem.length, coordinateSystem.length)

    elseif shapeKind == Modia3D.GridKind
        grid::Modia3D.Shapes.Grid = obj.shape
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(grid.axis, R_abs), SimVisGrid, id, DefaultMaterial,
                       grid.length, grid.width, grid.length; extras=@MVector[grid.distance, grid.lineWidth, 0.0])

    elseif shapeKind == Modia3D.SpringKind
        spring::Modia3D.Shapes.Spring = obj.shape
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(spring.axis, R_abs), SimVisSpring, id, obj.visualMaterial,
                       spring.length, spring.diameter, spring.diameter; extras=@MVector[spring.windings, spring.wireDiameter/2, 0.0])

    elseif shapeKind == Modia3D.GearWheelKind
        gearWheel::Modia3D.Shapes.GearWheel = obj.shape
        visualizeShape(simVis, r_abs, Shapes.rotateAxis2z(gearWheel.axis, R_abs), SimVisGearWheel, id, obj.visualMaterial,
                       gearWheel.diameter, gearWheel.diameter, gearWheel.length; extras=@MVector[gearWheel.innerDiameter/gearWheel.diameter, gearWheel.teeth, gearWheel.angle*180/pi])

    elseif shapeKind == Modia3D.ModelicaKind
        modelica::Modia3D.Shapes.ModelicaShape = obj.shape
        visualizeShape(simVis, r_abs, R_abs, ShapeType(modelica.type), id, obj.visualMaterial,
                       modelica.lengthX, modelica.lengthY, modelica.lengthZ; extras=modelica.extra)

    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.Shapes.FileMesh = obj.shape
        SimVis_setFileObject(simVis, id, Cint(0), r_abs, R_abs,
                             MVector{3,Float64}(fileMesh.scaleFactor), Cint(obj.visualMaterial.reflectslight), obj.visualMaterial.shininess, obj.visualMaterial.transparency, Cint(obj.visualMaterial.wireframe), Cint(0),
                             fileMesh.filename, Cint(fileMesh.smoothNormals), fileMesh.useMaterialColor, MVector{3,Cint}(obj.visualMaterial.color),
                             Cint(obj.visualMaterial.shadowMask), emptyShaderName)

    elseif shapeKind == Modia3D.TextKind && simVis.isProfessionalEdition  # function setTextObject is not available in community edition
        textShape::Modia3D.Shapes.TextShape = obj.shape
        SimVis_setTextObject(simVis, id, Cint(textShape.axisAlignment), textShape.text, 0.0, Cint(0), r_abs, R_abs,
                             textShape.font.charSize, textShape.font.fontFileName, MVector{3,Cint}(textShape.font.color), textShape.font.transparency,
                             textShape.offset, Cint(textShape.alignment), Cint(0))

    end
    return nothing
end
