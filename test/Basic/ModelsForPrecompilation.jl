module ModelsForPrecompilation

using Modia3D

Dummy = Model3D(
    world  = Object3D(feature=Scene(provideAnimationHistory=true, enableVisualization=false,
                                    enableContactDetection=true,
                                    gravityField=UniformGravityField(g=9.81, n=[0, -1, 0]))),

    body1  = Object3D(feature=Solid(shape=Beam(axis=1, length=1.0, width=0.1, thickness=0.1),
                                    solidMaterial="Aluminium")),
    frame1 = Object3D(parent=:body1, translation=[-0.5, 0.0, 0.0]),
    rev    = Revolute(obj1=:world, obj2=:frame1),

    body2  = Object3D(feature=Solid(shape=Box(lengthX=1.0, lengthY=0.1, lengthZ=0.1), massProperties=MassProperties(mass=1.0))),
    prism  = Prismatic(obj1=:world , obj2=:body2, axis=[0.0, 20.0, 0.0]),

    body3  = Object3D(parent=:world, fixedToParent=:false,
                                    translation=[0.0, 1.0, 0.0],
                                    angularVelocity=[10.0, 0.0, -5.0],
                                    feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=VisualMaterial(color="Blue"),
                                    solidMaterial="Steel",
                                    massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                                    collision=true))
)

dummy = @instantiateModel(Dummy, unitless=true)
requiredFinalStates = [-0.01409972769199364, -7.0596959177668255e-6, -0.009809999999999998, -4.911840743732473e-6, 1.911668227391551e-18, 0.9999950881592562, -5.650237323663999e-19, 5.421091995435263e-15, -0.009810000000000728, -1.6124980940930281e-15, 0.009999952436232525, -2.5034486890006063e-5, -0.004999904940503696, 10.0, 0.0, -5.0]
simulate!(dummy, stopTime=0.001, interval=0.001, requiredFinalStates=requiredFinalStates)

end
