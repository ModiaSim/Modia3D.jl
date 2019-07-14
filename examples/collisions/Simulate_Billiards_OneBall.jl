"""
    module Simulate_Billiards_OneBall

Simulates a billiard ball on a (non-standard) square billiard table
that is hitting the cushing several times. This model is used to
test the collision with the cushing.
"""
module Simulate_Billiards_OneBall

using  Modia3D
import Modia3D.ModiaMath

vmatBalls = Material(color="Red"      , transparency=0.0)
vmatTable = Material(color="DarkGreen", transparency=0.4)

diameter = 0.06
radius   = diameter/2

TableX = 1.24
TableY = 1.12
TableZ = 0.03
LyBox  = 0.03
hz     = 7*radius/5
hzz    = hz/3

@assembly Cushion(world) begin
  cushionPart11 = Solid(SolidBox(LyBox, TableY, hz) , "BilliardCushion", vmatTable)
  cushionPart12 = Solid(SolidBox(1.5*LyBox, TableY, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial="BilliardCushion")
  cushionPart21 = Solid(SolidBox(TableX, LyBox , hz) , "BilliardCushion", vmatTable)
  cushionPart22 = Solid(SolidBox(TableX, 1.5*LyBox, hzz,rsmall=0.0) , "BilliardCushion", vmatTable; contactMaterial="BilliardCushion")

  cushion11 = Object3D(world    , cushionPart11, fixed=true, r=[TableX/2 - LyBox/2 , 0.0                 , hz/2])
  cushion12 = Object3D(cushion11, cushionPart12, fixed=true, r=[-LyBox/4           , 0.0                 , hz/2+hzz/2])
  cushion31 = Object3D(world    , cushionPart11, fixed=true, r=[-TableX/2 + LyBox/2, 0.0                 , hz/2])
  cushion32 = Object3D(cushion31, cushionPart12, fixed=true, r=[LyBox/4            , 0.0                 , hz/2+hzz/2])
  cushion21 = Object3D(world    , cushionPart21, fixed=true, r=[0.0                , TableY/2 - LyBox/2  , hz/2])
  cushion22 = Object3D(cushion21, cushionPart22, fixed=true, r=[0.0                , -LyBox/4            , hz/2+hzz/2])
  cushion41 = Object3D(world    , cushionPart21, fixed=true, r=[0.0                , -TableY/2 + LyBox/2 , hz/2])
  cushion42 = Object3D(cushion21, cushionPart22, fixed=true, r=[0.0                , LyBox/4             , hz/2+hzz/2])
end


distance_balls = sqrt(3)/2*diameter
dist           = 0.0001
solidBall      = Solid(SolidSphere(diameter), "BilliardBall", vmatBalls ; contactMaterial="BilliardBall")
solidTable     = Solid(SolidBox(TableX, TableY, TableZ), "BilliardTable", vmatTable; contactMaterial="BilliardTable")

@assembly Billiards_OneBall() begin
  world   = Object3D()
  cushion = Cushion(world)
  table   = Object3D(world, solidTable, fixed=true , r=[0.0, 0.0, -TableZ/2])
  ball    = Object3D(world, solidBall , fixed=false, r=[0.0, 0.0, diameter/2], v_start=[6.0, 3.0, 0.0], visualizeFrame=true)
end

gravField         = UniformGravityField(g=9.81, n=[0,0,-1])
billiards_OneBall = Billiards_OneBall(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=false,
                                                                defaultFrameLength=0.1, enableContactDetection=true))

# Modia3D.visualizeAssembly!( billiards_OneBall )
model = SimulationModel( billiards_OneBall, maxNumberOfSteps=1000 )   # default maxNumberOfSteps gives an error
# ModiaMath.print_ModelVariables(model)
result = ModiaMath.simulate!(model; stopTime=0.5, tolerance=1e-8, log=false)

ModiaMath.plot(result, [("ball.r[1]", "ball.r[2]"),
                        ("ball.v[1]", "ball.v[2]"),
                        ("ball.w[1]", "ball.w[2]")])

println("... success of examples/collisions/Simulate_Billiards_OneBall!")
end
