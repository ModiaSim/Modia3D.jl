module contactForceLaw_rollingBall_2balls

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatBalls = Modia3D.Material(color="Red" , transparency=0.5)
vmatTable = Modia3D.Material(color="Green" , transparency=0.5)

cmatTable = Modia3D.ElasticContactMaterial2("BilliardTable")
cmatBall = Modia3D.ElasticContactMaterial2("BilliardBall")

LxGround = 10.0
LyBox = 2.0
LzBox = 0.3
diameter = 0.06

@assembly Table(world) begin
  withBox = Modia3D.Solid(Modia3D.SolidBox(LxGround, LyBox, LzBox) , "BilliardTable", vmatTable; contactMaterial = cmatTable)
  box1 = Modia3D.Object3D(world, withBox, r=[0.0, 0.0, -LzBox/2], fixed=true)
end

@assembly RollingBall() begin
  world = Modia3D.Object3D(visualizeFrame=true)
  table = Table(world)
  ball1 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-4.8, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0] )
  ball2 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-3.0, 0.0, diameter/2])
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = RollingBall(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=true, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

#Modia3D.visualizeAssembly!( bill )

model = Modia3D.SimulationModel( bill )
result = ModiaMath.simulate!(model; stopTime=5.0, tolerance=1e-8,interval=0.0001, log=false)

ModiaMath.plot(result, [("ball1.r[1]", "ball2.r[1]"),
                        ("ball1.r[2]", "ball2.r[2]"),
                        ("ball1.r[3]", "ball2.r[3]"),
                        ("ball1.v[1]", "ball2.v[1]"),
                        ("ball1.v[3]", "ball2.v[3]"),
                        ("ball1.w[2]", "ball2.w[2]"),
                        ("ball1.w[1]", "ball2.w[1]")])

println("... success of contactForceLaw_rollingBall_2balls.jl!")
end
