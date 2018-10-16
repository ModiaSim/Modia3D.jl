module Collision_Bars

using Modia3D

import Modia3D.ModiaMath
using  Modia3D.StaticArrays
using  Modia3D.Unitful



vgreen  = Modia3D.Material(color="Green", transparency=0.5)
vyellow = Modia3D.Material(color="Yellow", transparency=0.5)
vred    = Modia3D.Material(color="Red", transparency=0.5)
cmat    = Modia3D.defaultContactMaterial()


@signal negMove() begin
   output = ModiaMath.RealScalar("output", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::negMove, sim::ModiaMath.SimulationState)
    signal.output.value  = -Modia3D.linearMovement(pi/3, sim.tStart, sim.tEnd, sim.time)
end

@signal posMove() begin
   output = ModiaMath.RealScalar("output", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::posMove, sim::ModiaMath.SimulationState)
   signal.output.value  = Modia3D.linearMovement(pi/3, sim.tStart, sim.tEnd, sim.time)
end


# Materials
massProperties = "Aluminium"
vmat1   = Modia3D.Material(color="LightBlue", transparency=0.5, wireframe=false)
vblue   = Modia3D.Material(color="LightBlue" , transparency=0.5)
vred    = Modia3D.Material(color="Red", transparency=0.5)
vgreen  = Modia3D.Material(color="Green", transparency=0.5)
vblack  = Modia3D.Material(color="Black", transparency=0.5)
vyellow = Modia3D.Material(color="Yellow", transparency=0.5)
vpink   = Modia3D.Material(color="Pink", transparency=0.5)
vgrey = Modia3D.Material(color="Grey", transparency=0.5)
cmat    = Modia3D.defaultContactMaterial()
vMatAABB = Modia3D.Material(color="Grey", transparency=0.95)
vrevolute    = Modia3D.Material(color="Red", transparency=0.0)

# dimensions
Dx = 0.2
Lx = 1.0
Ly = 0.3
Lz = 0.3

groundWidth  = 1.0
groundHeight = 0.01
cyl  = Modia3D.Cylinder(1.2*Ly, Ly/2)
Tcyl = ModiaMath.rot1(90u"°")
TGround = ModiaMath.rot1(-90u"°")



# Bounding Boxes AABB
@assembly BoundingBoxes(world) begin
  blau1AABB      = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
  blau2AABB      = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
  gruen1AABB     = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
  gruen2AABB     = Modia3D.Object3D(world,  Modia3D.Box(0.9,0.9,0.9      , material=vMatAABB))
end

@assembly greenAssembly(world) begin
  green1    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vgreen; contactMaterial = cmat),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  green2    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vgreen; contactMaterial = cmat),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  Modia3D.connect(green1.frames[2], green2.frames[1]; R=ModiaMath.rot3(45u"°"))
  world_f1 = Modia3D.Object3D(world , r=[0.5, 0.5, 0.0])
  revGruen    = Modia3D.Revolute(world_f1, green1.frames[1])
  posSig  = posMove()
  posSignal = Modia3D.SignalToFlangeAngle(posSig.output)
  Modia3D.connect(revGruen, posSignal)
  Modia3D.Object3D(world_f1,  Modia3D.Cylinder(0.2,0.31      , material=vrevolute))
end

#=
@assembly blueAssembly(world) begin
  blue1    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vblue; contactMaterial = cmat),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  blue2    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vgrey; contactMaterial = cmat),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  Modia3D.connect(blue1.frames[2], blue2.frames[1]; R=ModiaMath.rot3(-40u"°"))
  # world_f2 = Modia3D.Object3D(world , r=[0.5, 3.5, 0.0])
  Modia3D.Object3D(blue1.frames[1],  Modia3D.Cylinder(0.2,0.31      , material=vrevolute))
  Modia3D.Object3D(blue2.frames[1],  Modia3D.Cylinder(0.2,0.31      , material=vrevolute))
end
=#


@assembly blueAssembly(world) begin
  blue1    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vblue; contactMaterial = cmat),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  blue2    = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vblue; contactMaterial = cmat),[ [-Lx/2,0,0],[ Lx/2,0,0] ])
  Modia3D.connect(blue1.frames[2], blue2.frames[1]; R=ModiaMath.rot3(-45u"°"))
  world_f2 = Modia3D.Object3D(world , r=[0.5, 3.5, 0.0])
  revBlau  = Modia3D.Revolute(world_f2, blue1.frames[1])
  negSig  = negMove()
  negSignal = Modia3D.SignalToFlangeAngle(negSig.output)
  Modia3D.connect(revBlau, negSignal)
  Modia3D.Object3D(world_f2,  Modia3D.Cylinder(0.2,0.3      , material=vrevolute))
end


@assembly collision() begin
  world = Modia3D.Object3D() # Modia3D.CoordinateSystem(1.0), visualizeFrame=true
  gruen    = greenAssembly(world)
  blau     = blueAssembly(world)
  boundingBoxes   = BoundingBoxes(world)

  #revBlau  = Modia3D.Revolute(gruen.green2.frames[2], blau.blue1.frames[1])
  #negSig  = negMove()
  #Modia3D.connectSignals(revBlau, :phi, negSig, me)
end
coll = collision(sceneOptions= Modia3D.SceneOptions(visualizeFrames=false,defaultFrameLength=0.3,enableContactDetection=true, nz_max = 3))

# Kinematic simulation
Modia3D.initAnalysis!(coll)

AABB = coll._internal.scene.options.contactDetection.contactPairs.AABB


tStart = 0.0
tEnd   = 2*pi

@static if VERSION >= v"0.7.0-DEV.2005"
    LINSPACE(start,stop,length) = range(0.0, stop=stop, length=length)
else
    LINSPACE(start,stop,length) = linspace(start,stop,length)
end

for time =  LINSPACE(tStart, tEnd, 101)
  Modia3D.update!(coll, time, tStart, tEnd)
  Modia3D.selectContactPairs!(coll)
  # Modia3D.getDistances!(coll)


  coll.boundingBoxes.blau1AABB.data.Lx = abs(AABB[1][1].x_max - AABB[1][1].x_min)
  coll.boundingBoxes.blau1AABB.data.Ly = abs(AABB[1][1].y_max - AABB[1][1].y_min)
  coll.boundingBoxes.blau1AABB.data.Lz = abs(AABB[1][1].z_max - AABB[1][1].z_min)
  coll.boundingBoxes.blau1AABB.r_abs   = coll.blau.blue1.frame0.r_abs

  coll.boundingBoxes.blau2AABB.data.Lx = abs(AABB[1][2].x_max - AABB[1][2].x_min)
  coll.boundingBoxes.blau2AABB.data.Ly = abs(AABB[1][2].y_max - AABB[1][2].y_min)
  coll.boundingBoxes.blau2AABB.data.Lz = abs(AABB[1][2].z_max - AABB[1][2].z_min)
  coll.boundingBoxes.blau2AABB.r_abs   = coll.blau.blue2.frame0.r_abs

  coll.boundingBoxes.gruen1AABB.data.Lx = abs(AABB[2][1].x_max - AABB[2][1].x_min)
  coll.boundingBoxes.gruen1AABB.data.Ly = abs(AABB[2][1].y_max - AABB[2][1].y_min)
  coll.boundingBoxes.gruen1AABB.data.Lz = abs(AABB[2][1].z_max - AABB[2][1].z_min)
  coll.boundingBoxes.gruen1AABB.r_abs   = coll.gruen.green1.frame0.r_abs

  coll.boundingBoxes.gruen2AABB.data.Lx = abs(AABB[2][2].x_max - AABB[2][2].x_min)
  coll.boundingBoxes.gruen2AABB.data.Ly = abs(AABB[2][2].y_max - AABB[2][2].y_min)
  coll.boundingBoxes.gruen2AABB.data.Lz = abs(AABB[2][2].z_max - AABB[2][2].z_min)
  coll.boundingBoxes.gruen2AABB.r_abs   = coll.gruen.green2.frame0.r_abs


  Modia3D.setComputationFlag(coll)
  Modia3D.visualize!(coll,time)
end
Modia3D.closeAnalysis!(coll)


println("... success of Collision_Bars.jl!")
end
