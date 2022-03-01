"""
    module TwoCollidingBalls

Simulates a ball that is first sliding, then rolling and then colliding
with another ball on a table.
"""
module TwoCollidingBalls

using Modia3D

vmatSolids = VisualMaterial(color="Red"  , transparency=0.0)
vmatTable  = VisualMaterial(color="Green", transparency=0.1)

LxTable  = 3.0
LyTable  = 0.5
LzTable  = 0.02
diameter = 0.06

collidingBalls = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                            enableContactDetection=true)),

    table = Object3D(parent=:world,
                     translation=[0.0, 0.0, -LzTable/2],
                     feature=Solid(shape=Box(lengthX=LxTable, lengthY=LyTable, lengthZ=LzTable),
                                   solidMaterial="DryWood",
                                   visualMaterial=vmatTable,
                                   contactMaterial="BilliardTable",
                                   collision=true)),

    ball1 = Object3D(feature=Solid(shape=Sphere(diameter=diameter),
                                   solidMaterial="BilliardBall",
                                   visualMaterial=vmatSolids,
                                   collision=true)),
    joint1 = FreeMotion(obj1=:world, obj2=:ball1,
                        r=Var(init=Modia.SVector{3,Float64}(-1.3, 0.0, diameter/2)),
                        v=Var(init=Modia.SVector{3,Float64}(3.0, 0.0, 0.0)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0))),

    ball2 = Object3D(feature=Solid(shape=Sphere(diameter=diameter),
                                   solidMaterial="BilliardBall",
                                   visualMaterial=vmatSolids,
                                   collision=true)),
    joint2 = FreeMotion(obj1=:world, obj2=:ball2,
                        r=Var(init=Modia.SVector{3,Float64}(0.0, 0.0, diameter/2)),
                        rot=Var(init=Modia.SVector{3,Float64}(pi/2, 0.0, 0.0)))
)

twoCollidingBalls = @instantiateModel(collidingBalls, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.5
tolerance = 1e-6
requiredFinalStates = [0.4489974852019629, 4.971482124288075e-6, 0.029996861274764766, 0.5336096206429409, 4.775435383850775e-6, -8.566849073992856e-7, 1.5708013764495665, 1.2298784775382833e-5, -55.4441088791202, -6.408457808447407e-5, -9.894396552324208e-5, -17.78647176388158, 1.4032405177409832, -4.464837913922324e-6, 0.029997744288682248, 1.4383823207541608, -4.574293232521244e-6, -2.3103945635420407e-6, 1.570796533109225, 1.2115305686199998e-8, -44.07171252097457, 0.00015344214722251976, 2.5704782290946986e-6, -47.9463685248874]
simulate!(twoCollidingBalls, stopTime=stopTime, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(twoCollidingBalls, ["joint1.r" "joint1.rot"; "joint1.v" "joint1.w"], figure=1)

end
