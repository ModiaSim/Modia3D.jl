module contactForceLaw_Billard

using Modia3D
using Modia3D.StaticArrays
import Modia3D.ModiaMath


vmatBalls = Modia3D.Material(color="Red" , transparency=0.5)
vmatTable = Modia3D.Material(color="Green" , transparency=0.5)

cmatTable   = Modia3D.ElasticContactMaterial2("BilliardTable")
cmatBall    = Modia3D.ElasticContactMaterial2("BilliardBall")
cmatCushion = Modia3D.ElasticContactMaterial2("BilliardCushion")

diameter = 0.06

TableX = 2.24
TableY = 1.12
TableZ = 0.05
LyBox = 0.05
LzBox = 0.05


@assembly Table(world) begin
  box = Modia3D.Solid(Modia3D.SolidBox(TableX, TableY, TableZ) , "BilliardTable", vmatTable; contactMaterial = cmatBall)
  table = Modia3D.Object3D(world, box, r=[0.0, 0.0, -TableZ/2], fixed=true)
end

@assembly Cushion(world) begin
  # breite = TableY
  withBox = Modia3D.Solid(Modia3D.SolidBox(LyBox, TableY, LzBox) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  lengthBox = Modia3D.Solid(Modia3D.SolidBox(TableX, LyBox, LzBox) , "BilliardCushion", vmatTable; contactMaterial = cmatCushion)
  cushion1 = Modia3D.Object3D(world, withBox, r=[TableX/2, 0.0, TableZ/2], fixed=true)
  cushion2 = Modia3D.Object3D(world, withBox, r=[-TableX/2, 0.0, TableZ/2], fixed=true)
  cushion3 = Modia3D.Object3D(world, lengthBox, r=[0.0, TableY/2, TableZ/2], fixed=true)
  cushion4 = Modia3D.Object3D(world, lengthBox, r=[0.0, -TableY/2, TableZ/2], fixed=true)
end


@assembly Billard1() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = Table(world)
  cushion = Cushion(world)
ball1 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-0.8, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0] )

#  ball1 = Modia3D.Object3D(world, Modia3D.Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial = cmatBall), fixed = false, r=[-0.8, 0.0, 1+diameter/2], v_start=[0.0, 0.0, 0.0] )

#  startBall = StartBall(world)
#=
  ball1 = BillardBall(world, 2.0, 0.0)

  ball2 = BillardBall(world, 2.5, 0.251)
  ball3 = BillardBall(world, 2.5, -0.251)

  ball4 = BillardBall(world, 3.0, 0.0)
  ball5 = BillardBall(world, 3.0, 0.501)
  ball6 = BillardBall(world, 3.0, -0.501)

  ball7 = BillardBall(world, 3.5, 0.251)
  ball8 = BillardBall(world, 3.5, 0.752)
  ball9 = BillardBall(world, 3.5, -0.251)
  ball10 = BillardBall(world, 3.5, -0.752)

  ball11 = BillardBall(world, 4.0, 0.0)
  ball12 = BillardBall(world, 4.0, 0.501)
  ball13 = BillardBall(world, 4.0, 1.002)
  ball14= BillardBall(world, 4.0, -0.501)
  ball15= BillardBall(world, 4.0, -1.002)
  =#
end

#=
@assembly Billard2() begin
  world = Modia3D.Object3D(visualizeFrame=false)
  table = BillardTable(world)
  startBall = StartBall(world)

  ball1 = BillardBall(world, 2.0, 0.0)

  ball2 = BillardBall(world, 2.5, 0.25)
  ball3 = BillardBall(world, 2.5, -0.25)

  ball4 = BillardBall(world, 3.0, 0.0)
  ball5 = BillardBall(world, 3.0, 0.5)
  ball6 = BillardBall(world, 3.0, -0.5)

  ball7 = BillardBall(world, 3.5, 0.25)
  ball8 = BillardBall(world, 3.5, 0.75)
  ball9 = BillardBall(world, 3.5, -0.25)
  ball10 = BillardBall(world, 3.5, -0.75)

  ball11 = BillardBall(world, 4.0, 0.0)
  ball12 = BillardBall(world, 4.0, 0.5)
  ball13 = BillardBall(world, 4.0, 1.0)
  ball14= BillardBall(world, 4.0, -0.5)
  ball15= BillardBall(world, 4.0, -1.0)
end
=#

gravField = Modia3D.UniformGravityField(g=9.81, n=[0,0,-1])
bill = Billard1(sceneOptions=Modia3D.SceneOptions(gravityField=gravField,visualizeFrames=false, defaultFrameLength=0.2,nz_max = 100, enableContactDetection=true, visualizeContactPoints=false, visualizeSupportPoints=false))

# Modia3D.visualizeAssembly!( bill )


model = Modia3D.SimulationModel( bill )
# ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=2.0, tolerance=1e-8,interval=0.001, log=false)

ModiaMath.plot(result, [("ball1.r[1]"),
                        ("ball1.r[2]"),
                        ("ball1.r[3]"),
                        ("ball1.v[1]"),
                        ("ball1.v[3]"),
                        ("ball1.w[2]"),
                        ("ball1.w[1]")])


println("... success of contactForceLaw_Billard.jl!")
end
