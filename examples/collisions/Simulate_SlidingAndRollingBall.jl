"""
    module Simulate_SlidingAndRollingBall

Simulates a ball that is first sliding and then rolling on a table.
"""
module Simulate_SlidingAndRollingBall

using  Modia3D
import Modia3D.ModiaMath


vmatSolids   = Material(color="Red"       , transparency=0.0)    # material of solids
vmatTable    = Material(color="Green"     , transparency=0.1)    # material of table

LxTable  = 3.0
LyTable  = 0.5
LzTable  = 0.02
diameter = 0.06

solidBall  = Solid(SolidSphere(diameter), "BilliardBall", vmatSolids; contactMaterial="BilliardBall")
solidTable = Solid(SolidBox(LxTable, LyTable, LzTable) , "BilliardTable", vmatTable; contactMaterial = "BilliardTable")


@assembly RollingBall() begin
  world = Object3D()
  table = Object3D(world, solidTable, fixed=true , r=[1.5, 0.0, -LzTable/2])
  ball  = Object3D(world, solidBall , fixed=false, r=[0.2, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0], visualizeFrame=true )
end

gravField   = UniformGravityField(g=9.81, n=[0,0,-1])
rollingBall = RollingBall(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                                    defaultFrameLength=0.1, enableContactDetection=true))

# Modia3D.visualizeAssembly!( rollingBall )
model = SimulationModel(rollingBall)

result = ModiaMath.simulate!(model; stopTime=0.5, tolerance=1e-8, log=false)

ModiaMath.plot(result, [ "ball.r[1]"  "ball.v[1]"
                         "ball.r[3]"  "ball.w[2]"])

println("... success of examples/collisions/Simulate_SlidingAndRollingBall.jl!")
end
