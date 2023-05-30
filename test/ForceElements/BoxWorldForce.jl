module BoxWorldForceSimulation

using Modia3D

function forceVec(; time::Float64, axis::Int64, maxForce::Float64)::Modia.SVector{3,Float64}
    if time < 1.0
        frc = 0.0
    elseif time < 2.0
        frc = maxForce*(time - 1.0)
    elseif time < 3.0
        frc = maxForce
    elseif time < 4.0
        frc = maxForce*(4.0 - time)
    else
        frc = 0.0
    end
    if axis == 1
        return Modia.SVector{3,Float64}(frc, 0.0, 0.0)
    elseif axis == 2
        return Modia.SVector{3,Float64}(0.0, frc, 0.0)
    else
        return Modia.SVector{3,Float64}(0.0, 0.0, frc)
    end
end
forceVector(; time, objectApply, objectCoord) = forceVec(; time=time, axis=1, maxForce=1.0)

BoxWorldForce = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=NoGravityField(),
                                   nominalLength=:(2*Length))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    dirFrame = Object3D(parent=:world,
                        rotation=[0.0, 0.0, 30.0/180*pi],
                        feature=Visual(shape=CoordinateSystem(length=:(0.75*Length)))),
    box = Object3D(parent=:world, fixedToParent=false,
                   feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    force = WorldForce(;objectApply=:box,
                       forceFunction=forceVector,
                       objectCoord=:dirFrame)
)

boxWorldForce = @instantiateModel(BoxWorldForce, unitless=true, logCode=false)

stopTime = 5.0
dtmax = 0.1
requiredFinalStates = [4.330091998452314, 2.4999797809222875, 0.0, 1.7320345346635413, 0.9999906048337232, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
simulate!(boxWorldForce, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(boxWorldForce, ["box.translation", "box.velocity", "force.forceVector"], figure=1)

end
