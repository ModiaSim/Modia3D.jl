module CollidingCylindersSimulation

using Modia3D

vmatRed   = VisualMaterial(color="Red")
vmatGreen = VisualMaterial(color="Green")
vmatBlue  = VisualMaterial(color="Blue")
vmatGrey  = VisualMaterial(color="Grey", transparency=0.5)

LxTable = 2.0
LyTable = 2.0
LzTable = 0.02
diameterCyl      = 0.1
lengthCyl        = 0.5
innerDiameterCyl = 0.08

collidingCylinders = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=lengthCyl,
                                   enableContactDetection=true)),
    cylinderX = Object3D(parent=:world, fixedToParent=false,
                         translation=[-0.3, -0.1, 0.5],
                         rotation=[0.0, -90*u"°", -80*u"°"],
                         feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatRed,
                                       collision=true)),
    cylinderY = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.0, -0.1, 0.5],
                         rotation=[-10*u"°", 0.0, 0.0],
                         feature=Solid(shape=Cylinder(axis=2, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGreen,
                                       collision=true)),
    cylinderZ = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.3, -0.1, 0.5],
                         rotation=[80*u"°", 0.0, 0.0],
                         feature=Solid(shape=Cylinder(axis=3, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatBlue,
                                       collision=true)),
    cylinder2 = Object3D(parent=:world,
                         translation=[0.0, 0.1, 0.2],
                         feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=1.0),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGrey,
                                       collision=true)),
    cylinder3 = Object3D(parent=:world,
                         translation=[0.0, -0.1, 0.0],
                         feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=1.0),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGrey,
                                       collision=true))
)

CollidingCylinders = @instantiateModel(collidingCylinders, unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 0.7
interval = 0.001
tolerance = 1e-8
finalStates = [-0.3, -0.7970361906203778, 0.008588033707431024, -7.237169810640399e-24, -1.6896514746046425, -1.8362931543566983, 5.311030412261472, 3.273267080119862e-13, -1.5707963267949006, 12.501897002117005, -6.119100356078922e-14, 8.97384406112105e-14, -6.4344239781983074e-24, -0.7958169253113232, 0.009907524790588372, -9.132826064767804e-28, -1.6857839839384585, -1.8338319584815206, 3.739172631603938, -2.2304263793016205e-13, -2.2968456547063965e-15, 12.491667707530048, 3.8752399136944956e-14, -7.288167098788829e-14, 0.3, -0.7972350189036318, 0.008404752508151168, 4.172205601393931e-27, -1.6904048029264813, -1.8367363258987721, 5.309290153368781, 7.938841542342687e-16, -8.112382530020162e-14, 12.494972632285036, 2.02760099086996e-14, -2.895778704265324e-14]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("cylinderX.translation", "cylinderY.translation", "cylinderZ.translation") ("cylinderX.rotation", "cylinderY.rotation", "cylinderZ.rotation")
                          ("cylinderX.velocity", "cylinderY.velocity", "cylinderZ.velocity") ("cylinderX.angularVelocity", "cylinderY.angularVelocity", "cylinderZ.angularVelocity")], figure=1)

end
