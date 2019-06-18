module contactForceLaw_rollingBall_2balls

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatGraphics = Modia3D.Material(color="LightBlue" , transparency=0.5)    # material of Graphics
vmatSolids = Modia3D.Material(color="Red" , transparency=0.5)         # material of solids
#c = 1e9, d = 100.0

# coefficientOfRestitution, slidingFrictionCoefficient, rotationalFrictionCoefficient
# steel org: 0.7, 0.5, 0.001
# funkt cor=0.9,  mu_k = 0.1, mu_r = 0.1,
# funkt cor=0.9, mu_k = 0.1
# funkt cor=0.9, mu_k = 0.2
# cor=0.8, mu_k = 0.1
# cor=0.9, mu_k = 0.1, mu_r = 0.1

# funkt nicht cor=0.9, mu_k = 0.3, mu_r = 0.01
#
# cor=0.8, mu_k = 0.1, mu_r = 0.025
cmatTable = Modia3D.ElasticContactMaterial(name="Steel", E=2.0e8,  cor=0.1, mu_r = 0.1 ) #E=2.0e7, 
cmatBall = Modia3D.ElasticContactMaterial(name="BillardBall", cor=0.9, mu_k = 0.2, mu_r = 0.1)

LxGround = 10.0
LyBox = 2.0
LzBox = 0.3
diameter = 0.06
massBall = (diameter/2)^3 *pi*4/3 * 1768.0 # kg
println("massBall = ", massBall)
@assembly Table(world) begin
  withBox = Modia3D.Solid(Modia3D.SolidBox(LxGround, LyBox, LzBox) , "Steel", vmatSolids; contactMaterial = cmatTable)
  box1 = Modia3D.Object3D(world, withBox, r=[0.0, 0.0, -LzBox/2], fixed=true)
end

# Modia3D.MassProperties(m=massBall)

@assembly StartBall(world) begin
  sphere = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter) ,  "BillardBall" , vmatSolids ; contactMaterial = cmatBall), fixed = false, r=[-4.8, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0] )
end

@assembly Ball(world) begin  # , xPos, yPos, zPos
  sphere = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter) ,"BillardBall", vmatSolids ; contactMaterial = cmatBall), fixed = false, r=[-3.0, 0.0, diameter/2])
end


@assembly RollingBall() begin
  world     = Modia3D.Object3D(visualizeFrame=false)
  table     = Table(world)
  startBall = StartBall(world)
  ball      = Ball(world)
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = RollingBall(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.7,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

#Modia3D.visualizeAssembly!( bill )

model = Modia3D.SimulationModel( bill )
ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=2.0, tolerance=1e-8,interval=0.001, log=false)

ModiaMath.plot(result, ["startBall.sphere.r[3]", "startBall.sphere.v[3]","startBall.sphere.w", "ball.sphere.v","ball.sphere.w"])


println("... success of contactForceLaw_rollingBall_2balls.jl!")
end
