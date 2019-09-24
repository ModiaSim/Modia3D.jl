module Billard_Balls16

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
dist = 0.0001

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
  cushion31 = Modia3D.Object3D(world    , cushionPart11, r=[-TableX/2 + LyBox/2, 0.0, hz/2], fixed=true)
  cushion32 = Modia3D.Object3D(cushion31, cushionPart12, r=[LyBox/4     , 0.0, hz/2+hzz/2], fixed=true)
  cushion21 = Modia3D.Object3D(world    , cushionPart21, r=[0.0, TableY/2 - LyBox/2 , hz/2], fixed=true)
  cushion22 = Modia3D.Object3D(cushion21, cushionPart22, r=[0.0, -LyBox/4 , hz/2+hzz/2], fixed=true)
  cushion41 = Modia3D.Object3D(world    , cushionPart21, r=[0.0, -TableY/2 + LyBox/2 , hz/2], fixed=true)
  cushion42 = Modia3D.Object3D(cushion41, cushionPart22, r=[0.0, LyBox/4 , hz/2+hzz/2], fixed=true)
end


@assembly BillardBall(world, xPos, yPos) begin
  ball = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[xPos, yPos, diameter/2])
end

@assembly Billard() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = Table(world)
  cushion = Cushion(world)
  ballStart = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-0.8, -0.1, diameter/2], v_start=[3.0, 0.1, 0.0])

  ball1 = BillardBall(world, TableX/6, 0.0)

  ball2  = BillardBall(world, TableX/6 + 1*distance_balls + dist,  1/2*(diameter+dist))
  ball3  = BillardBall(world, TableX/6 + 1*distance_balls + dist, -1/2*(diameter+dist))

  ball4  = BillardBall(world, TableX/6 + 2*(distance_balls+dist),  (diameter+dist))
  ball5  = BillardBall(world, TableX/6 + 2*(distance_balls+dist),  0.0)
  ball6  = BillardBall(world, TableX/6 + 2*(distance_balls+dist), -(diameter+dist))

  ball7  = BillardBall(world, TableX/6 + 3*(distance_balls+dist),  3/2*(diameter+dist))
  ball8  = BillardBall(world, TableX/6 + 3*(distance_balls+dist),  1/2*(diameter+dist))
  ball9  = BillardBall(world, TableX/6 + 3*(distance_balls+dist), -1/2*(diameter+dist))
  ball10 = BillardBall(world, TableX/6 + 3*(distance_balls+dist), -3/2*(diameter+dist))

  ball11 = BillardBall(world, TableX/6 + 4*(distance_balls+dist),  2*(diameter+dist))
  ball12 = BillardBall(world, TableX/6 + 4*(distance_balls+dist),  (diameter+dist))
  ball13 = BillardBall(world, TableX/6 + 4*(distance_balls+dist),  0.0)
  ball14 = BillardBall(world, TableX/6 + 4*(distance_balls+dist), -(diameter+dist))
  ball15 = BillardBall(world, TableX/6 + 4*(distance_balls+dist), -2*(diameter+dist))
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billard(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

model = Modia3D.SimulationModel( bill )
result = ModiaMath.simulate!(model; stopTime=5.0, tolerance=1e-8,interval=0.0001, log=false)


println("... success of Billard_Balls16.jl!")
end
