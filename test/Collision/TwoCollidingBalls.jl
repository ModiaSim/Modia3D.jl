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

    ball1 = Object3D(parent=:world, fixedToParent=false,
                     translation=[-1.3, 0.0, diameter/2],
                     velocity=[3.0, 0.0, 0.0],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=Solid(shape=Sphere(diameter=diameter),
                                   solidMaterial="BilliardBall",
                                   visualMaterial=vmatSolids,
                                   collision=true)),

    ball2 = Object3D(parent=:world, fixedToParent=false,
                     translation=[0.0, 0.0, diameter/2],
                     rotation=[pi/2, 0.0, 0.0],
                     feature=Solid(shape=Sphere(diameter=diameter),
                                   solidMaterial="BilliardBall",
                                   visualMaterial=vmatSolids,
                                   collision=true)),
)

twoCollidingBalls = @instantiateModel(collidingBalls, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 1.5
tolerance = 1e-6
dtmax = 0.1
if Sys.iswindows()
    requiredFinalStates = [0.44876394694456134, 9.305927636126645e-17, 0.029996866914584027, 0.5333704927224155, 9.191631858558948e-17, -8.536332325885959e-7, 1.5707963267948966, 2.428034767780229e-16, -55.43736210378611, -3.0637882098742386e-15, 17.778500349018145, 1.6854528233546344e-19, 1.4034679368692125, -2.5752631917689518e-14, 0.029997749668811288, 1.4386201962066418, -2.635051209881685e-14, -2.3079771375263612e-6, 1.570796326794897, -1.816097264237583e-14, -44.078687623522335, 8.783557043687735e-13, 47.95429595521847, 0.0]
else
    requiredFinalStates = missing
end
simulate!(twoCollidingBalls, stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(twoCollidingBalls, ["ball1.translation" "ball1.rotation"; "ball1.velocity" "ball1.angularVelocity"], figure=1)
plot(twoCollidingBalls, ["ball2.translation" "ball2.rotation"; "ball2.velocity" "ball2.angularVelocity"], figure=2)

end
