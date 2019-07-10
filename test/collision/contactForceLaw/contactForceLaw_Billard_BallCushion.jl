module contactForceLaw_Billard_BallCushion

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatBalls = Modia3D.Material(color="Red" , transparency=0.0)
vmatTable = Modia3D.Material(color="DarkGreen" , transparency=0.6)

cmatTable   = Modia3D.ElasticContactMaterial2("BilliardTable")
cmatBall    = Modia3D.ElasticContactMaterial2("BilliardBall")
cmatCushion = Modia3D.ElasticContactMaterial2("BilliardCushion")

diameter = 0.06
radius = diameter/2

TableX = 2.24
TableY = 1.12
TableZ = 0.05
LyBox = 0.05
hz  = 7*radius/5
hzz = hz/3


@assembly Table(world) begin
  box = Modia3D.Solid(Modia3D.SolidBox(TableX, TableY, TableZ) , "BilliardTable", vmatTable; contactMaterial = cmatTable)
  table = Modia3D.Object3D(world, box, r=[0.0, 0.0, -TableZ/2], fixed=true)
end

@assembly Cushion(world) begin
  cushionPart11 = Modia3D.Solid(Modia3D.SolidBox(LyBox, TableY, hz) , "BilliardCushion", vmatTable)
  cushionPart12 = Modia3D.Solid(Modia3D.SolidBox(2.0*LyBox, TableY, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)

  cushion11 = Modia3D.Object3D(world    , cushionPart11, r=[TableX/2, 0.0, hz/2], fixed=true)
  cushion12 = Modia3D.Object3D(cushion11, cushionPart12, r=[0.0     , 0.0, hz/2+hzz/2], fixed=true)
end

#=
@assembly BillardBall(world, xPos, yPos) begin
  ball = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[xPos, yPos, diameter/2])
end
=#

distance_balls = sqrt(3)/2*diameter

dist = 0.0001

@assembly Billiard() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = Table(world)
  cushion = Cushion(world)
  ball1 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-0.5, 0.2, diameter/2], v_start=[6.0, 0.0, 0.0], visualizeFrame=true)
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billiard(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( bill )


model = Modia3D.SimulationModel( bill )
# ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=0.9, tolerance=1e-8,interval=0.001, log=true)

ModiaMath.plot(result, [("ball1.r[1]"),
                        ("ball1.v[1]"),
                        ("ball1.w[2]")])
                        # ("ball1.r[3]"), ("ball1.v[3]"),


println("... success of contactForceLaw_Billard_BallCushion.jl!")
end
