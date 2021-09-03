module BouncingSphere3D

using Modia

BouncingSphere = Model(
    boxHeigth = 0.1,
    world      = Object3D(feature=Scene(enableContactDetection = true, # default value
                        animationFile="BouncingSphere.json")),
    ground     = Object3D(parent=:world, translation=:[0.0,-boxHeigth/2,0.0],
                    feature=Solid(shape=Box(lengthX=4.0, lengthY=:boxHeigth, lengthZ=0.7),
                        visualMaterial=VisualMaterial(color="DarkGreen"),
                        solidMaterial="Steel", # for mass and force computation
                        collision=true)),      # enable collision flag
    sphere     = Object3D(feature=Solid(shape=Sphere(diameter=0.2),
                        visualMaterial=VisualMaterial(color="Blue"),
                        solidMaterial="Steel", # for mass and force computation
                        massProperties=MassPropertiesFromShapeAndMass(mass=0.001),
                        collision=true)),      # enable collision flag
    free       = FreeMotion(obj1=:world, obj2=:sphere, r=Var(init=[0.0, 1.0, 0.0]))
)

bouncingSphere = @instantiateModel(buildModia3D(BouncingSphere), unitless=true)
simulate!(bouncingSphere, stopTime=2.2, dtmax=0.1)

@usingModiaPlot
plot(bouncingSphere, "free.r", figure=1)

end
