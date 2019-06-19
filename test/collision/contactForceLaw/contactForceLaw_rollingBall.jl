module contactForceLaw_rollingBall

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatGraphics = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of Graphics
vmatSolids = Modia3D.Material(color="Red" , transparency=0.5)         # material of solids
#c = 1e9, d = 100.0

cmatTable = Modia3D.ElasticContactMaterial(name="DryWood",  mu_r = 0.1)
cmatBall = Modia3D.ElasticContactMaterial(name="BillardBall", cor=0.9, mu_k = 0.2, mu_r = 0.1)


LxGround = 10.0
LyBox = 2.0
LzBox = 0.3
diameter = 0.06


@assembly Table(world) begin
  #ground = Modia3D.Object3D(world, Modia3D.Box(LxGround, LyGround, LzGround; material=vmatGraphics) )
  withBox = Modia3D.Solid(Modia3D.SolidBox(LxGround, LyBox, LzBox) , "DryWood", vmatSolids; contactMaterial = cmatTable)
  box1 = Modia3D.Object3D(world, withBox, r=[0.0, 0.0, -LzBox/2], fixed=true)
end

@assembly StartBall(world) begin
  sphere = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter) , "BillardBall" , vmatSolids ; contactMaterial = cmatBall), fixed = false, r=[-4.8, 0.0, diameter/2], v_start=[3.0, 0.001, 0.0] )
end

@assembly RollingBall() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = Table(world)
  ball = StartBall(world)
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = RollingBall(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=true, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( bill )

model = Modia3D.SimulationModel( bill )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=4.0, tolerance=1e-8,interval=0.001, log=false)

ModiaMath.plot(result, ["ball.sphere.r[1]","ball.sphere.r[2]","ball.sphere.r[3]", "ball.sphere.v[1]","ball.sphere.v[2]","ball.sphere.v[3]", "ball.sphere.w[1]","ball.sphere.w[2]","ball.sphere.w[3]"])


println("... success of contactForceLaw_rollingBall.jl!")
end
