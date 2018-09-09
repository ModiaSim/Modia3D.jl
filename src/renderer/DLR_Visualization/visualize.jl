# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

@enum ShapeType box=1 sphere=2 cylinder=3 cone=4 capsule=5 coordsys=6 spring=7 gearwheel=8 pipe=9 grid=10 beam=11


const mvecSize  = MVector{3,Cdouble}(0.0,0.0,0.0)

@inline function visualizeShape(frame::Composition.Object3D, shape::ShapeType, id::Ptr{NOTHING}, material::Graphics.Material, 
                                Lx::Float64, Ly::Float64, Lz::Float64, extras::MVector{3,Float64}=Basics.ZeroMVector)
  mvecSize[1] = Lx
  mvecSize[2] = Ly
  mvecSize[3] = Lz

  SimVis_setBaseObject(id, Cint(0), Cint(shape), frame.r_abs, frame.R_abs, mvecSize, material.color, Cint(material.wireframe),
                       Cint(material.reflectslight), material.shininess, extras, material.transparency, Cint(0), Cint(material.shadowMask))
end

visualizeSphere(data::Graphics.Sphere, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, sphere, id, data.material, data.Dx, data.Dx, data.Dx)
visualizeSolidSphere(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, sphere, id, data.material, data.geo.Dx, data.geo.Dx, data.geo.Dx)
#
visualizeEllipsoid(data::Graphics.Ellipsoid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, sphere, id, data.material, data.Dx, data.Dy, data.Dz)
visualizeSolidEllipsoid(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, sphere, id, data.material, data.geo.Dx, data.geo.Dy, data.geo.Dz)
#
visualizeBox(data::Graphics.Box, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, box, id, data.material, data.Lx, data.Ly, data.Lz)
visualizeSolidBox(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, box, id, data.material, data.geo.Lx, data.geo.Ly, data.geo.Lz)
#
visualizeCylinder(data::Graphics.Cylinder, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, cylinder, id, data.material, data.Dx, data.Dy, data.Lz)
visualizeSolidCylinder(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, cylinder, id, data.material, data.geo.Dx, data.geo.Dy, data.geo.Lz)
#
visualizeCone(data::Graphics.Cone, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, cone, id, data.material, data.Dx, data.Dy, data.Lz, data.extras)
visualizeSolidCone(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, cone, id, data.material, data.geo.Dx, data.geo.Dy, data.geo.Lz, data.geo.extras)
#
visualizeCapsule(data::Graphics.Capsule, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, capsule, id, data.material, data.Dx, data.Dy, data.Lz)
visualizeSolidCapsule(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, capsule, id, data.material, data.geo.Dx, data.geo.Dy, data.geo.Lz)
#
visualizeSpring(data::Graphics.Spring, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, spring, id, data.material, data.Dx, data.Dy, data.Lz, data.extras)
#
visualizeGearWheel(data::Graphics.GearWheel, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, gearwheel, id, data.material, data.Dx, data.Dy, data.Lz, data.extras)
#
visualizePipe(data::Graphics.Pipe, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, pipe, id, data.material, data.Dx, data.Dy, data.Lz, data.extras)
visualizeSolidPipe(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, pipe, id, data.material, data.geo.Dx, data.geo.Dy, data.geo.Lz, data.geo.extras)
#
visualizeBeam(data::Graphics.Beam, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, beam, id, data.material, data.Lx,data.Dy, data.Lz)
visualizeSolidBeam(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, beam, id, data.material, data.geo.Lx,data.geo.Dy, data.geo.Lz)
#
const DefaultMaterial = Graphics.Material()
visualizeCoordinateSystem(data::Graphics.CoordinateSystem, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, coordsys, id, DefaultMaterial, data.Lx, data.Lx, data.Lx)

#
visualizeGrid(data::Graphics.Grid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
          visualizeShape(frame, grid, id, DefaultMaterial, data.Lx, data.Ly, data.Lx, data.extras)
#
const emptyShaderName=" "
visualizeFileMesh(data::Graphics.FileMesh, frame::Composition.Object3D, id::Ptr{NOTHING}) =
     SimVis_setFileObject(id, Cint(0), frame.r_abs, frame.R_abs,
          data.scaleFactor, Cint(data.material.reflectslight), data.material.shininess, data.material.transparency, Cint(data.material.wireframe), Cint(0),
          data.filename, Cint(data.smoothNormals), data.useMaterialColor, MVector{3,Cint}(data.material.color),
          Cint(data.material.shadowMask), emptyShaderName)
#
visualizeSolidFileMesh(data::Solids.Solid, frame::Composition.Object3D, id::Ptr{NOTHING}) =
     SimVis_setFileObject(id, Cint(0), frame.r_abs, frame.R_abs,
          data.geo.scaleFactor, Cint(data.material.reflectslight), data.material.shininess, data.material.transparency, Cint(data.material.wireframe), Cint(0),
          data.geo.filename, Cint(data.geo.smoothNormals), data.geo.useGraphicsMaterialColor, MVector{3,Cint}(data.material.color),
          Cint(data.material.shadowMask), emptyShaderName)
