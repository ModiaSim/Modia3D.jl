module TwoCollidingBoxes

using Modia

vmat1 = VisualMaterial(color="Green", transparency=0.6)
vmat2 = VisualMaterial(color="Red"  , transparency=0.6)
cmat = "Steel"

collidingBoxes = Model(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[-1, 0, 0]), mprTolerance = 1.0e-9,
                                          enableContactDetection=true)),

    fixedBox = Object3D(parent=:world,
                        translation=[-3.0, 0.0, -2.5],
                        feature=Solid(shape=Box(lengthX=3.0, lengthY=2.0, lengthZ=5.0),
                                      solidMaterial="Aluminium",
                                      visualMaterial=vmat1,
                                      contactMaterial=cmat,
                                      collision=true,
                                      collisionSmoothingRadius=0.001)),

    movingBox = Object3D(feature=Solid(shape=Box(lengthX=0.5, lengthY=0.5, lengthZ=0.5),
                                       solidMaterial="Aluminium",
                                       visualMaterial=vmat2,
                                       contactMaterial=cmat,
                                       collision=true,
                                       collisionSmoothingRadius=0.001)),
    movingBoxJoint = FreeMotion(obj1=:world, obj2=:movingBox,
                                r=Var(init=[0.0, 0.0, 0.0]),
                                rot=Var(init=[pi/2, 0.0, 0.0]))
)

twoCollidingBoxes = @instantiateModel(buildModia3D(collidingBoxes), unitless=true, log=false, logStateSelection=false, logCode=false)


stopTime = 2.0
tolerance = 1e-6
interval = 0.001
requiredFinalStates = [-1.2486715374142026, 6.290497915662845e-7, 0.0659365516539737, 0.8477959270372105, 2.6616962347476056e-6, 0.5739231232863249, 1.5707961606494825, 1.638506827004496e-7, 0.18102076812097648, 0.00013271162847691188, 1.9332870388332806e-5, 1.645887052972817]

#if Sys.iswindows()
#    requiredFinalStates = [-1.2486715374142026, 6.290497915662845e-7, 0.0659365516539737, 0.8477959270372105, 2.6616962347476056e-6, 0.5739231232863249, 1.5707961606494825, 1.638506827004496e-7, 0.18102076812097648, 0.00013271162847691188, 1.9332870388332806e-5, 1.645887052972817]
#else
#    requiredFinalStates = [-1.2491609001458825, 3.4171296735804487e-7, 0.06689208772957456, 0.7494258087005188, 1.6721279542955506e-6, 0.5829025911027681, 1.5707788581459448, -3.2574807995825715e-6, 0.183216453113141, -0.00012373469585807555, -3.169934525839379e-5, 1.6744038834216513]
#end
simulate!(twoCollidingBoxes, stopTime=stopTime, tolerance=tolerance, interval=interval, log=true, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(twoCollidingBoxes, ["movingBoxJoint.r" "movingBoxJoint.rot"; "movingBoxJoint.v" "movingBoxJoint.w"], figure=1)

end
