module contactForceLaw_Billard

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatGraphics = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of Graphics
vmatSolids = Modia3D.Material(color="Red" , transparency=0.5)         # material of solids
#c = 1e9, d = 100.0

cmat = Modia3D.ElasticContactMaterial(name="Steel", mu_r = 0.0)

LxGround = 25.4
LyGround = 12.7
LzGround = 0.1
LyBox = 1.0
LzBox = 2.0
@assembly BillardTable(world) begin
  #ground = Modia3D.Object3D(world, Modia3D.Box(LxGround, LyGround, LzGround; material=vmatGraphics) )
  withBox = Modia3D.Solid(Modia3D.SolidBox(LxGround, LyBox, LzBox) , "Steel", vmatSolids; contactMaterial = cmat)
  lengthBox = Modia3D.Solid(Modia3D.SolidBox(LyBox, LyGround, LzBox) , "Steel", vmatSolids; contactMaterial = cmat)
  box1 = Modia3D.Object3D(world, withBox, r=[0.0, LyGround/2, LzGround], fixed=true)
  box2 = Modia3D.Object3D(world, withBox, r=[0.0, -LyGround/2, LzGround], fixed=true)
  box3 = Modia3D.Object3D(world, lengthBox, r=[LxGround/2, 0.0, LzGround], fixed=true)
  box4 = Modia3D.Object3D(world, lengthBox, r=[-LxGround/2, 0.0, LzGround], fixed=true)
end

diameter = 0.5
@assembly StartBall(world) begin
  sphere = Modia3D.Object3D( Modia3D.Solid(Modia3D.SolidSphere(diameter) , "Steel" , vmatSolids ; contactMaterial = cmat))
  helpFrame = Modia3D.Object3D(visualizeFrame=false)
  prisX = Modia3D.Prismatic(world, helpFrame; axis=1, v_start=6.0, s_start=-6.0, canCollide=true)
  prisY = Modia3D.Prismatic(helpFrame, sphere, axis=2, v_start=0.3, s_start=0.1, canCollide=true)
end

@assembly BillardBall(world, xPos, yPos) begin
  sphere = Modia3D.Object3D( Modia3D.Solid(Modia3D.SolidSphere(diameter) , "Steel" , vmatSolids ; contactMaterial = cmat))
  helpFrame = Modia3D.Object3D(visualizeFrame=false)
  prisX2 = Modia3D.Prismatic(world, helpFrame; axis=1, canCollide=true , s_start=xPos)
  prisY2 = Modia3D.Prismatic(helpFrame, sphere, axis=2, canCollide=true, s_start=yPos)
end


@assembly Billard1() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = BillardTable(world)
  startBall = StartBall(world)

  ball1 = BillardBall(world, 2.0, 0.0)

  ball2 = BillardBall(world, 2.5, 0.251)
  ball3 = BillardBall(world, 2.5, -0.251)

  ball4 = BillardBall(world, 3.0, 0.0)
  ball5 = BillardBall(world, 3.0, 0.501)
  ball6 = BillardBall(world, 3.0, -0.501)

  ball7 = BillardBall(world, 3.5, 0.251)
  ball8 = BillardBall(world, 3.5, 0.752)
  ball9 = BillardBall(world, 3.5, -0.251)
  ball10 = BillardBall(world, 3.5, -0.752)

  ball11 = BillardBall(world, 4.0, 0.0)
  ball12 = BillardBall(world, 4.0, 0.501)
  ball13 = BillardBall(world, 4.0, 1.002)
  ball14= BillardBall(world, 4.0, -0.501)
  ball15= BillardBall(world, 4.0, -1.002)
end


@assembly Billard2() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = BillardTable(world)
  startBall = StartBall(world)

  ball1 = BillardBall(world, 2.0, 0.0)

  ball2 = BillardBall(world, 2.5, 0.25)
  ball3 = BillardBall(world, 2.5, -0.25)

  ball4 = BillardBall(world, 3.0, 0.0)
  ball5 = BillardBall(world, 3.0, 0.5)
  ball6 = BillardBall(world, 3.0, -0.5)

  ball7 = BillardBall(world, 3.5, 0.25)
  ball8 = BillardBall(world, 3.5, 0.75)
  ball9 = BillardBall(world, 3.5, -0.25)
  ball10 = BillardBall(world, 3.5, -0.75)

  ball11 = BillardBall(world, 4.0, 0.0)
  ball12 = BillardBall(world, 4.0, 0.5)
  ball13 = BillardBall(world, 4.0, 1.0)
  ball14= BillardBall(world, 4.0, -0.5)
  ball15= BillardBall(world, 4.0, -1.0)
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billard2(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( bill )


model = Modia3D.SimulationModel( bill )
result = ModiaMath.simulate!(model; stopTime=17.0, tolerance=1e-8,interval=0.001, log=false)


println("... success of contactForceLaw_Billard.jl!")
end
