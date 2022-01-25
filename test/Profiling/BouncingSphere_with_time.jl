module BouncingSphere_with_time

using Modia

@time begin BouncingSphere = Model(
        boxHeigth = 0.1,
        groundMaterial = VisualMaterial(color="DarkGreen", transparency=0.5),
        gravField = UniformGravityField(g=9.81, n=[0, -1, 0]),
        world = Object3D(feature=Scene(gravityField=:gravField,
                                            visualizeFrames=false,
                                            defaultFrameLength=0.2,
                                            visualizeBoundingBox = true,
                                            enableContactDetection=true,
                                            visualizeContactPoints=false)),
        worldFrame = Object3D(parent=:world, feature=Visual(shape=CoordinateSystem(length=0.5))),
        ground = Object3D(parent=:world,
                        translation=:[0.0,-boxHeigth/2,0.0],
                        feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=0.7),
                                        visualMaterial=:groundMaterial,
                                        solidMaterial="Steel",
                                        collision=true)),
        sphere = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                                        visualMaterial=VisualMaterial(color="Blue"),
                                        solidMaterial="Steel",
                                        massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                                        collision=true)),
        prism = Prismatic(obj1=:world, obj2=:sphere, axis=2, s=Var(init=1.0))
    )
end

@time bouncingSphere = @instantiateModel(buildModia3D(BouncingSphere), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 2.2
dtmax = 0.1
tolerance = 1e-8
requiredFinalStates = [0.01948634300615831, 0.10092552754875266]
@time simulate!(bouncingSphere, stopTime=stopTime, tolerance=tolerance, dtmax=dtmax, log=false, logStates=false, logEvents=false, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
@time plot(bouncingSphere, ["prism.s", "prism.v"], figure=1)

end
