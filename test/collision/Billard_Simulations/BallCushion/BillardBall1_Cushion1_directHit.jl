module BillardBall1_Cushion1_directHit

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatBalls = Modia3D.Material(color="Red" , transparency=0.0)
vmatTable = Modia3D.Material(color="DarkGreen" , transparency=0.5)

cmatTable   = "BilliardTable"
cmatBall    = "BilliardBall"
cmatCushion = "BilliardCushion"

diameter = 0.06
radius = diameter/2
distance_balls = sqrt(3)/2*diameter

TableX = 2.24
TableY = 1.12
TableZ = 0.05
LyBox = 0.05
LzBox = 0.05
hz  = 7*radius/5
hzz = hz/3

@assembly Table(world) begin
  box = Modia3D.Solid(Modia3D.SolidBox(TableX, TableY, TableZ) , "BilliardTable", vmatTable; contactMaterial = cmatTable)
  table = Modia3D.Object3D(world, box, r=[0.0, 0.0, -TableZ/2], fixed=true)
end

@assembly Cushion(world) begin
  cushionPart11 = Modia3D.Solid(Modia3D.SolidBox(LyBox, TableY, hz) , "BilliardCushion", vmatTable)
  cushionPart12 = Modia3D.Solid(Modia3D.SolidBox(1.5*LyBox, TableY, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  cushionPart21 = Modia3D.Solid(Modia3D.SolidBox(TableX, LyBox , hz) , "BilliardCushion", vmatTable)
  cushionPart22 = Modia3D.Solid(Modia3D.SolidBox(TableX, 1.5*LyBox, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)

  cushion11 = Modia3D.Object3D(world    , cushionPart11, r=[TableX/2 - LyBox/2, 0.0, hz/2], fixed=true)
  cushion12 = Modia3D.Object3D(cushion11, cushionPart12, r=[-LyBox/4     , 0.0, hz/2+hzz/2], fixed=true)
end


@assembly BillardBall(world, xPos, yPos) begin
  ball = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[xPos, yPos, diameter/2])
end


@assembly Billard1() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = Table(world)
  cushion = Cushion(world)
  ballStart = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-0.8, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0])
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billard1(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

model = Modia3D.SimulationModel( bill )
result = ModiaMath.simulate!(model; stopTime=1.2, tolerance=1e-8,interval=0.0001, log=false)

ModiaMath.plot(result, [("ballStart.r[1]"),
                        ("ballStart.r[2]"),
                        ("ballStart.r[3]"),
                        ("ballStart.v[1]"),
                        ("ballStart.v[3]"),
                        ("ballStart.w[2]"),
                        ("ballStart.w[1]")])

println("... success of BillardBall1_Cushion1_directHit.jl!")
end
