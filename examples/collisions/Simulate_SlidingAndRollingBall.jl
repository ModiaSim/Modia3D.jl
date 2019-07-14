"""
    module Simulate_SlidingAndRollingBall

Simulates a ball that is first sliding and then rolling on a table.
"""
module Simulate_SlidingAndRollingBall

using  Modia3D
import Modia3D.ModiaMath


vmatGraphics = Material(color="LightBlue" , transparency=0.5)    # material of Graphics
vmatSolids   = Material(color="Red"       , transparency=0.0)    # material of solids
vmatTable    = Material(color="Green"     , transparency=0.1)    # material of table

LxTable  = 3.0
LyTable  = 0.5
LzTable  = 0.02
diameter = 0.06

solidBall  = Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatSolids; contactMaterial="BilliardBall")
solidTable = Solid(SolidBox(LxTable, LyTable, LzTable) , "DryWood", vmatTable; contactMaterial = "BilliardTable")

@assembly RollingBall() begin
  world = Object3D()
  table = Object3D(world, solidTable, fixed=true , r=[1.5, 0.0, -LzTable/2])
  ball  = Object3D(world, solidBall , fixed=false, r=[0.2, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0], visualizeFrame=true )
end

gravField   = UniformGravityField(g=9.81, n=[0,0,-1])
rollingBall = RollingBall(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                                    defaultFrameLength=0.1, enableContactDetection=true))
#Modia3D.visualizeAssembly!( rollingBall )
model = SimulationModel( rollingBall )
#ModiaMath.print_ModelVariables(model)

#=
using PyPlot
using PyCall

pyplot_rc = PyCall.PyDict(PyPlot.matplotlib."rcParams")
pyplot_rc["font.family"]      = "sans-serif"
pyplot_rc["font.sans-serif"]  = ["Calibri", "Arial", "Verdana", "Lucida Grande"]
pyplot_rc["font.size"]        = 12.0
pyplot_rc["lines.linewidth"]  = 1.5
pyplot_rc["grid.linewidth"]   = 0.5
pyplot_rc["axes.grid"]        = true
pyplot_rc["axes.titlesize"]   = "medium"
pyplot_rc["figure.titlesize"] = "medium"
=#

result = ModiaMath.simulate!(model; stopTime=0.5, tolerance=1e-8, log=false)

#=
clf()
fig, ax = PyPlot.subplots(figsize=(3,9))

ModiaMath.plot(result, [("ball1.r[1]"),
                        ("ball1.r[3]"),
                        ("ball1.v[1]"),
                        ("ball1.w[2]")],
                        figure=1, reuse=true)
=#

ModiaMath.plot(result, [ "ball.r[1]"  "ball.v[1]"
                         "ball.r[3]"  "ball.w[2]"])

println("... success of examples/collisions/Simulate_SlidingAndRollingBall.jl!")
end
