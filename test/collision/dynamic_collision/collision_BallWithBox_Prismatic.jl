module collision_BallWithBox_Prismatic

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatGraphics = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of Graphics
vmatSolids = Modia3D.Material(color="Red" , transparency=0.5)         # material of solids

cmat = "Steel"

LxGround = 16.0
LyGround = 8.0
LzGround = 0.1
LyBox = 1.0
LzBox = 1.0

diameter = 1.0

@assembly Billard() begin
  world = Modia3D.Object3D(visualizeFrame=true)
  lengthBox = Modia3D.Solid(Modia3D.SolidBox(LyBox, LyGround, LzBox) , "Steel", vmatSolids; contactMaterial = cmat)
  box4 = Modia3D.Object3D(world, lengthBox, r=[-LxGround/2, 0.0, LzGround], fixed=true)

  sphere = Modia3D.Object3D( Modia3D.Solid(Modia3D.SolidSphere(diameter) , "Steel" , vmatSolids ; contactMaterial = cmat))
  prisX = Modia3D.Prismatic(world, sphere; axis=1, canCollide = true)
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[-1,0,0])
bill = Billard(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=true))

model = Modia3D.SimulationModel( bill )
result = ModiaMath.simulate!(model; stopTime=1.5, tolerance=1e-8,interval=0.001, log=false)


println("... success of collision_BallWithBox_Prismatic.jl!")
end
