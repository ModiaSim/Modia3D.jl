module ZeroCrossingIssue

using Modia3D

# The penetration depth is in the order of 1e-11. With a tolerance of 1e-8, the penetration depth
# is not computed precisely enough, and computation time is excessive because the Jacobian is computed
# very often. This is fixed by setting elasticContactReductionFactor=1e-5.

ZeroCrossing = Model3D(
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=0.1,
                                   enableContactDetection=true,
                                   elasticContactReductionFactor=1e-5,
                                   maximumContactDamping=1e3)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=0.05))),
    ground = Object3D(parent=:world,
                      translation=:[0.0, 0.0, -0.005],
                      feature=Solid(shape=Box(lengthX=0.2, lengthY=0.2, lengthZ=0.01),
                                    visualMaterial=VisualMaterial(color="Grey75", transparency=0.5),
                                    solidMaterial="DryWood",
                                    collision=true)),
    ellipsoid = Object3D(parent=:world, fixedToParent=false,
                         translation=[0.01, 0.01, 0.01],
                         feature=Solid(shape=Ellipsoid(lengthX=0.1, lengthY=0.02, lengthZ=0.02),
                                       visualMaterial=VisualMaterial(color="Blue"),
                                       solidMaterial="DryWood",
                                       collision=true))
)

zeroCrossing = @instantiateModel(ZeroCrossing, unitless=true)

stopTime = 0.01
tolerance = 1e-8
requiredFinalStates = missing
simulate!(zeroCrossing, stopTime=stopTime, tolerance=tolerance, log=true, logEvents=false, logStates=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(zeroCrossing, ["ellipsoid.translation"; "ellipsoid.velocity"], figure=1)

end
