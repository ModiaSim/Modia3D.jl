module Billard_Balls4_Prismatic

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


@assembly Cushion(world) begin
  cushionPart11 = Modia3D.Solid(Modia3D.SolidBox(LyBox, TableY, hz) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  cushionPart12 = Modia3D.Solid(Modia3D.SolidBox(1.5*LyBox, TableY, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  cushionPart21 = Modia3D.Solid(Modia3D.SolidBox(TableX, LyBox , hz) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  cushionPart22 = Modia3D.Solid(Modia3D.SolidBox(TableX, 1.5*LyBox, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)

  cushion11 = Modia3D.Object3D(world    , cushionPart11, r=[TableX/2 - LyBox/2, 0.0, hz/2-radius/2], fixed=true)
  cushion12 = Modia3D.Object3D(cushion11, cushionPart12, r=[-LyBox/4     , 0.0, hz/2+hzz/2], fixed=true)
  cushion31 = Modia3D.Object3D(world    , cushionPart11, r=[-TableX/2 + LyBox/2, 0.0, hz/2-radius/2], fixed=true)
  cushion32 = Modia3D.Object3D(cushion31, cushionPart12, r=[LyBox/4     , 0.0, hz/2+hzz/2], fixed=true)
  cushion21 = Modia3D.Object3D(world    , cushionPart21, r=[0.0, TableY/2 - LyBox/2 , hz/2-radius/2], fixed=true)
  cushion22 = Modia3D.Object3D(cushion21, cushionPart22, r=[0.0, -LyBox/4 , hz/2+hzz/2], fixed=true)
  cushion41 = Modia3D.Object3D(world    , cushionPart21, r=[0.0, -TableY/2 + LyBox/2 , hz/2-radius/2], fixed=true)
  cushion42 = Modia3D.Object3D(cushion41, cushionPart22, r=[0.0, LyBox/4 , hz/2+hzz/2], fixed=true)
end

@assembly BillardBall(world, xPos, yPos) begin
  ball = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall))
  helpFrame = Modia3D.Object3D(visualizeFrame=false)
  prisX2 = Modia3D.Prismatic(world, helpFrame; axis=1, canCollide=true , s_start=xPos)
  prisY2 = Modia3D.Prismatic(helpFrame, ball,  axis=2, canCollide=true, s_start=yPos)
end

@assembly Billard() begin
  world = Modia3D.Object3D(visualizeFrame=false)
#  table = Table(world)
  cushion = Cushion(world)
  ballStart = Modia3D.Object3D( Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall))
  helpFrame = Modia3D.Object3D(visualizeFrame=false)
  prisX = Modia3D.Prismatic(world, helpFrame; axis=1, v_start=3.0, s_start=-0.8, canCollide=true)
  prisY = Modia3D.Prismatic(helpFrame, ballStart, axis=2, v_start=0.1, s_start=-0.1, canCollide=true)

  ball1 = BillardBall(world, TableX/6, 0.0)
  ball2  = BillardBall(world, TableX/6 + 1*distance_balls + dist,  1/2*(diameter+dist))
  ball3  = BillardBall(world, TableX/6 + 1*distance_balls + dist, -1/2*(diameter+dist))
end

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billard(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

model = Modia3D.SimulationModel( bill )
result = ModiaMath.simulate!(model; stopTime=5.0, tolerance=1e-8,interval=0.0001, log=false)

println("... success of Billard_Balls4_Prismatic.jl!")
end
