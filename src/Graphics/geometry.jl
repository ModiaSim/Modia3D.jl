# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Graphics (Modia3D/Graphics/_module.jl)
#
# Content:
#   This file contains mutable visual geometries that are only used for animation
#   (immutable solid geometries that have a volume are defined in module Modia3D.Solids.
#    They can be additionally used for collision handling and other purposes).

mutable struct Sphere <: Modia3D.AbstractGeometry
   Dx::Float64
   material::Material

   function Sphere(Dx::Float64; material=Material())
      @assert(Dx >= 0.0)
      new(Dx, material)
   end
end



mutable struct Ellipsoid <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Dz::Float64
  material::Material

  function Ellipsoid(Dx::Float64,Dy::Float64,Dz::Float64; material=Material())
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Dz >= 0.0)
    new(Dx,Dy,Dz,material)
  end
end



mutable struct Box <: Modia3D.AbstractGeometry
  Lx::Float64
  Ly::Float64
  Lz::Float64
  material::Material

  function Box(Lx::Float64,Ly::Float64,Lz::Float64; material=Material())
    @assert(Lx >= 0.0)
    @assert(Ly >= 0.0)
    @assert(Lz >= 0.0)
    new(Lx,Ly,Lz,material)
  end
end


mutable struct Cylinder <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  material::Material

  function Cylinder(Dx::Float64, Lz::Float64; Dy=Dx, material=Material())
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    new(Dx,Dy,Lz,material)
  end
end



mutable struct Capsule <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  material::Material

  function Capsule(Dx::Float64, Lz::Float64; Dy=Dx, material=Material())
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    new(Dx,Dy,Lz,material)
  end
end



mutable struct Beam <: Modia3D.AbstractGeometry
  Lx::Float64
  Dy::Float64
  Lz::Float64
  material::Material

  function Beam(Lx::Float64,Dy::Float64,Lz::Float64; material=Material())
    @assert(Lx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    new(Lx,Dy,Lz,material)
  end
end



mutable struct Cone <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  extras::MVector{3,Float64}
  material::Material

  function Cone(Dx::Float64, Lz::Float64; Dy=Dx,relativeTipDiameter=0.5,relativeInnerDiameter=1, material=Material())
    # Tip diameter >= 0.0, Tip diameter is relative to base diameter (0 = real cone, 1 = cylinder),
    # Inner diameter any real number, Inner diameter of the cone is proportional to tip size (0..1)
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(relativeTipDiameter>=0.0 && relativeTipDiameter<=1.0)
    @assert(relativeInnerDiameter>=0.0)
    extras=[relativeTipDiameter,relativeInnerDiameter,0.0]
    new(Dx,Dy,Lz,extras,material)
  end
end



mutable struct Pipe <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  extras::MVector{3,Float64}
  material::Material

  function Pipe(Dx::Float64, Lz::Float64; Dy=Dx, relativeInnerDiameter=0.9,relativeTipDiameter=1.0, material=Material())
    #relativeInnerDiameter ... Inner diameter of pipe (0 = rigid, 1 = fully hollow), every real number is allowed
    #relativeTipDiameter ... Tip diameter of pipe relative to base diameter (0 = real cone, 1 = cylinder), every real number >= 0 is possible
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(relativeTipDiameter>=0.0)
    extras=[0.0,relativeInnerDiameter,relativeTipDiameter]
    new(Dx,Dy,Lz,extras,material)
  end
end



mutable struct Spring <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  extras::MVector{3,Float64}
  material::Material

  function Spring(Dx::Float64, Lz::Float64; Dy=Dx,windings=5,springRadius=0.02, material=Material())
    # windings ... Winding number of the spring, integer > 0
    # springRadius ... Radius of spring wire > 0.0
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(isinteger(windings))
    @assert(windings>0.0)
    @assert(springRadius>0.0)
    extras=[windings,springRadius,0.0]
    new(Dx,Dy,Lz,extras,material)
  end
end



mutable struct GearWheel <: Modia3D.AbstractGeometry
  Dx::Float64
  Dy::Float64
  Lz::Float64
  extras::MVector{3,Float64}
  material::Material

  function GearWheel(Dx::Float64, Lz::Float64; Dy=Dx,relativeInnerDiameter=1,teeth=20,angle=0, material=Material())
    # relativeInnerDiameter ... Ratio of inner/outer radius of gearwheel, proportional to radius of gearwheel (0..1)
    # teeth ... Number of gearwheel teeth (negative teeth number for inner gearwheel (not 0)
    # angle ... Angle for beveled gearwheels (0 = cylindrical gearwheel)
    @assert(Dx >= 0.0)
    @assert(Dy >= 0.0)
    @assert(Lz >= 0.0)
    @assert(isinteger(teeth))
    @assert(teeth!=0.0)
    extras=[relativeInnerDiameter,teeth,angle]
    new(Dx,Dy,Lz,extras,material)
  end
end



mutable struct CoordinateSystem <: Modia3D.AbstractVisualElement
  Lx::Float64

  function CoordinateSystem(Lx::Float64)
    @assert(Lx >= 0.0)
    new(Lx)
  end
end



mutable struct Grid <: Modia3D.AbstractVisualElement
  Lx::Float64
  Ly::Float64
  extras::MVector{3,Float64}

  function Grid(Lx::Float64, Ly::Float64; distance=0.05, lineWidth=1.0)
    # distance ... Distance or grid points in x- and in y-direction
    # lineWidth ... Line width of the grid
    @assert(Lx >= 0.0)
    @assert(Ly >= 0.0)
    @assert(distance > 0.0)
    @assert(lineWidth > 0.0)
    extras=[distance,lineWidth,0.0]
    new(Lx,Ly,extras)
  end
end



mutable struct FileMesh <: Modia3D.AbstractGeometry
   filename::AbstractString
   scaleFactor::MVector{3,Float64}
   useMaterialColor::Bool
   smoothNormals::Bool
   material::Material

   function FileMesh(filename::AbstractString; scaleFactor=Basics.onesMVector()  , useMaterialColor::Bool=false, smoothNormals::Bool=false, material=Material())
      @assert(isfile(filename))
      @assert(scaleFactor[1] >= 0.0)
      @assert(scaleFactor[2] >= 0.0)
      @assert(scaleFactor[3] >= 0.0)
      new(filename, scaleFactor, useMaterialColor, smoothNormals, material)
   end
end
FileMesh(filename::AbstractString, scale::Number; useMaterialColor::Bool=false, smoothNormals::Bool=false, material=Material()) =
         FileMesh(filename; scaleFactor=[scale,scale,scale], useMaterialColor=useMaterialColor, smoothNormals=smoothNormals, material=material)
