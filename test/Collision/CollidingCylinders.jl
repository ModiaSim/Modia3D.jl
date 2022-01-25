module CollidingCylindersSimulation

using  Unitful
using Modia

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

collidingCylinders = Model(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   enableContactDetection=true)),

    cylinderX = Object3D(feature=Solid(shape=Cylinder(axis=1, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatRed,
                                       collision=true)),
    jointX = FreeMotion(obj1=:world, obj2=:cylinderX,
                        r=Var(init=[-0.3, -0.1, 0.5]),
                        rot=Var(init=[0.0, -90*u"°", -80*u"°"])),

    cylinderY = Object3D(feature=Solid(shape=Cylinder(axis=2, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatGreen,
                                       collision=true)),
    jointY = FreeMotion(obj1=:world, obj2=:cylinderY,
                        r=Var(init=[0.0, -0.1, 0.5]),
                        rot=Var(init=[-10*u"°", 0.0, 0.0])),

    cylinderZ = Object3D(feature=Solid(shape=Cylinder(axis=3, diameter=diameterCyl, length=lengthCyl, innerDiameter=innerDiameterCyl),
                                       solidMaterial="BilliardBall",
                                       visualMaterial=vmatBlue,
                                       collision=true)),
    jointZ = FreeMotion(obj1=:world, obj2=:cylinderZ,
                        r=Var(init=[0.3, -0.1, 0.5]),
                        rot=Var(init=[80*u"°", 0.0, 0.0])),

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

CollidingCylinders = @instantiateModel(buildModia3D(collidingCylinders), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 0.7
interval = 0.001
tolerance = 1e-8
finalStates =  [-0.3, -0.7912951104703625, 0.014315465297405685, 1.5382263697205537e-23, -1.6713227668023118, -1.8252495552795023, 5.311528246103376, 1.597580600135156e-13, -1.570796326794889, 4.0900087176554086e-13, -1.9772045460615305e-12, -12.479847941496443, 7.206645749967858e-28, -1.5466821874242411, -1.103414585697127, 1.6519032497807758e-27, -3.3589465482926086, -5.512089283857012, -3.838789677220436, -2.7897755369078287e-14, -4.585971148915582e-16, -12.998755672760737, -6.026512055478026e-14, 3.712976826003701e-13, 0.3, -0.7913074991067811, 0.01430529121118472, -1.0636566187082562e-26, -1.6713691919342737, -1.8252737771503114, 5.311422962861477, -2.4913547303307028e-15, -5.1268212661135167e-14, 12.479429217577563, 6.349958791483419e-13, -1.295519398855759e-13]
simulate!(CollidingCylinders, stopTime=stopTime, interval=interval, tolerance=tolerance, log=true, logStates=false, logEvents=false, requiredFinalStates=finalStates)

@usingModiaPlot
plot(CollidingCylinders, [("jointX.r", "jointY.r", "jointZ.r") ("jointX.rot", "jointY.rot", "jointZ.rot")
                                    ("jointX.v", "jointY.v", "jointZ.v") ("jointX.w", "jointY.w", "jointZ.w")], figure=1)

end
