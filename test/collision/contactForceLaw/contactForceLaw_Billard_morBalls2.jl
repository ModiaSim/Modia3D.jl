module contactForceLaw_Billard_moreBalls

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatBalls = Modia3D.Material(color="Red" , transparency=0.0)
vmatTable = Modia3D.Material(color="DarkGreen" , transparency=0.5)

cmatTable   = Modia3D.ElasticContactMaterial2("BilliardTable")
cmatBall    = Modia3D.ElasticContactMaterial2("BilliardBall")
cmatCushion = Modia3D.ElasticContactMaterial2("BilliardCushion")

diameter = 0.06

TableX = 2.24
TableY = 1.12
TableZ = 0.05
LyBox = 0.05
LzBox = 0.05

@assembly Cushion(world) begin
  # breite = TableY
  withBox = Modia3D.Solid(Modia3D.SolidBox(LyBox, TableY, LzBox) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  lengthBox = Modia3D.Solid(Modia3D.SolidBox(TableX, LyBox, LzBox) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  cushion1 = Modia3D.Object3D(world, withBox, r=[TableX/2, 0.0, TableZ/2], fixed=true)
  cushion2 = Modia3D.Object3D(world, withBox, r=[-TableX/2, 0.0, TableZ/2], fixed=true)
  cushion3 = Modia3D.Object3D(world, lengthBox, r=[0.0, TableY/2, TableZ/2], fixed=true)
  cushion4 = Modia3D.Object3D(world, lengthBox, r=[0.0, -TableY/2, TableZ/2], fixed=true)
end

billardBall    = Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall)
box            = Modia3D.Solid(Modia3D.SolidBox(TableX, TableY, TableZ) , "BilliardTable", vmatTable; contactMaterial = cmatTable)
distance_balls = sqrt(3)/2*diameter

@assembly Billard1() begin
  world     = Modia3D.Object3D(visualizeFrame=false)
  table     = Modia3D.Object3D(world, box, r=[0.0, 0.0, -TableZ/2], fixed=true)
  cushion   = Cushion(world)
  ballStart = Modia3D.Object3D(world, billardBall, fixed=false, r=[-0.8, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0])
  ball1     = Modia3D.Object3D(world, billardBall, fixed=false, r=[TableX/6, 0.0, diameter/2])
  ball2     = Modia3D.Object3D(world, billardBall, fixed=false, r=[TableX/6 + 1*distance_balls, 0.0,  diameter/2])
  ball3     = Modia3D.Object3D(world, billardBall, fixed=false, r=[TableX/6 + 1*distance_balls, 0.0, -diameter/2])


#=
  ball4  = BillardBall(world, TableX/6 + 2*distance_balls,  diameter)
  ball5  = BillardBall(world, TableX/6 + 2*distance_balls,  0.0)
  ball6  = BillardBall(world, TableX/6 + 2*distance_balls, -diameter)

  ball7  = BillardBall(world, TableX/6 + 3*distance_balls,  3*diameter/2)
  ball8  = BillardBall(world, TableX/6 + 3*distance_balls,  diameter/2)
  ball9  = BillardBall(world, TableX/6 + 3*distance_balls, -diameter/2)
  ball10 = BillardBall(world, TableX/6 + 3*distance_balls, -3*diameter/2)

  ball11 = BillardBall(world, TableX/6 + 4*distance_balls,  2*diameter)
  ball12 = BillardBall(world, TableX/6 + 4*distance_balls,  diameter)
  ball13 = BillardBall(world, TableX/6 + 4*distance_balls,  0.0)
  ball14 = BillardBall(world, TableX/6 + 4*distance_balls, -diameter)
  ball15 = BillardBall(world, TableX/6 + 4*distance_balls, -2*diameter)
  =#
end


gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billard1(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( bill )


model = Modia3D.SimulationModel( bill )
# ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=4.0, tolerance=1e-8,interval=0.001, log=true)

ModiaMath.plot(result, [("ball1.r[1]"),
                        ("ball1.r[2]"),
                        ("ball1.r[3]"),
                        ("ball1.v[1]"),
                        ("ball1.v[3]"),
                        ("ball1.w[2]"),
                        ("ball1.w[1]")])


println("... success of contactForceLaw_Billard_moreBalls.jl!")
end
