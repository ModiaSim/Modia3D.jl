"""
    module Simulate_TwoCollidingBalls

Simulates a ball that is first sliding, then rolling and then colliding
with another ball on a table.
"""
module Simulate_TwoRollingAndCollidingBalls

using Modia3D
import Modia3D.ModiaMath

vmatSolids   = Material(color="Red"       , transparency=0.0)    # material of solids
vmatTable    = Material(color="Green"     , transparency=0.1)    # material of table

LxTable  = 3.0
LyTable  = 0.5
LzTable  = 0.02
diameter = 0.06

solidBall  = Solid(Modia3D.SolidSphere(diameter), "BilliardBall", vmatSolids ; contactMaterial="BilliardBall")
solidTable = Solid(SolidBox(LxTable, LyTable, LzTable) , "DryWood", vmatTable; contactMaterial="BilliardTable")

@assembly TwoCollidingBalls() begin
  world = Object3D()
  table = Object3D(world, solidTable, fixed=true , r=[1.5, 0.0, -LzTable/2])
  ball1 = Object3D(world, solidBall , fixed=false, r=[0.2, 0.0, diameter/2], v_start=[3.0, 0.0, 0.0], visualizeFrame=true)
  ball2 = Object3D(world, solidBall , fixed=false, r=[1.5, 0.0, diameter/2], visualizeFrame=true)
end

gravField         = UniformGravityField(g=9.81, n=[0,0,-1])
twoCollidingBalls = TwoCollidingBalls(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                                                defaultFrameLength=0.1, enableContactDetection=true))

model = SimulationModel( twoCollidingBalls )

result = ModiaMath.simulate!(model; stopTime=1.0, tolerance=1e-8, log=false)

ModiaMath.plot(result, [ ("ball1.r[1]", "ball2.r[1]"),
                         ("ball1.v[1]", "ball2.v[1]"),
                         ("ball1.w[2]", "ball2.w[2]")])

println("... success of examples/collisions/Simulate_TwoCollidingBalls.jl!")
end
