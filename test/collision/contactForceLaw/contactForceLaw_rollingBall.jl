module contactForceLaw_rollingBall

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatGraphics = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of Graphics
vmatSolids = Modia3D.Material(color="Red" , transparency=0.5)         # material of solids
#c = 1e9, d = 100.0

cmat = Modia3D.ElasticContactMaterial(name="Steel", mu_r = 0.1)

LxGround = 20.0
LyGround = 30.0
LzGround = 0.1
LyBox = 4.0
LzBox = 1.0
@assembly Table(world) begin
  #ground = Modia3D.Object3D(world, Modia3D.Box(LxGround, LyGround, LzGround; material=vmatGraphics) )
  withBox = Modia3D.Solid(Modia3D.SolidBox(LxGround, LyBox, LzBox) , "Steel", vmatSolids; contactMaterial = cmat)
  box1 = Modia3D.Object3D(world, withBox, r=[0.0, 0.0, -0.75], fixed=true)
end

diameter = 0.5
@assembly StartBall(world) begin
  sphere = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter) , "Steel" , vmatSolids ; contactMaterial = cmat), fixed = false, r=[-7.0, 0.0, 0.0], v_start=[6.0, 0.0, 0.0] )
#=
  helpFrame = Modia3D.Object3D(visualizeFrame=false)
  helpFrame1 = Modia3D.Object3D(visualizeFrame=false)
  prisX = Modia3D.Prismatic(world, helpFrame; axis=1, v_start=6.0, s_start=-6.0, canCollide=true)
  prisY = Modia3D.Prismatic(helpFrame, helpFrame1, axis=2, v_start=0.0, s_start=0.0, canCollide=true)
  prisZ = Modia3D.Prismatic(helpFrame1, sphere, axis=3, v_start=0.0, s_start=0.0, canCollide=true)
  =#
end


@assembly RollingBall() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = Table(world)
  startBall = StartBall(world)
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = RollingBall(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( bill )

model = Modia3D.SimulationModel( bill )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=10.1, tolerance=1e-6,interval=0.001, log=false)

ModiaMath.plot(result, ["startBall.sphere.r[1]","startBall.sphere.r[2]","startBall.sphere.r[3]", "startBall.sphere.v[1]","startBall.sphere.v[2]","startBall.sphere.v[3]"])


println("... success of contactForceLaw_rollingBall.jl!")
end
