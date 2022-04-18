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
    prism  = Prismatic(obj1=:world , obj2=:body2, axis=2),

    body3  = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                                    visualMaterial=VisualMaterial(color="Blue"),
                                    solidMaterial="Steel",
                                    massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                                    collision=true)),
    free   = FreeMotion(obj1=:world, obj2=:body3, r=Var(init=Modia.SVector{3,Float64}(0.0, 1.0, 0.0)),
                                                  w=Var(init=Modia.SVector{3,Float64}(10.0, 0.0, -5.0)))
)

dummy = @instantiateModel(Dummy, unitless=true)
simulate!(dummy, stopTime=0.001, interval=0.001)

end
