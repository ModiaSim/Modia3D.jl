module BoxWorldTorqueSimulation

using Modia3D

function torqueVec(; time::Float64, axis::Int64, maxTorque::Float64)::Modia.SVector{3,Float64}
    if time < 1.0
        trq = 0.0
    elseif time < 2.0
        trq = maxTorque*(time - 1.0)
    elseif time < 3.0
        trq = maxTorque
    elseif time < 4.0
        trq = maxTorque*(4.0 - time)
    else
        trq = 0.0
    end
    if axis == 1
        return Modia.SVector{3,Float64}(trq, 0.0, 0.0)
    elseif axis == 2
        return Modia.SVector{3,Float64}(0.0, trq, 0.0)
    else
        return Modia.SVector{3,Float64}(0.0, 0.0, trq)
    end
end
torqueVector(; time, objectApply, objectCoord) = torqueVec(; time=time, axis=2, maxTorque=1.0)

BoxWorldTorque = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=NoGravityField(),
                                   nominalLength=:(2*Length))),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    dirFrame = Object3D(parent=:world,
                        rotation=[0.0, 0.0, 60.0/180*pi],
                        feature=Visual(shape=CoordinateSystem(length=:(0.75*Length)))),
    box = Object3D(parent=:world, fixedToParent=false,
                   feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    torque = WorldTorque(objectApply=:box,
                         torqueFunction=torqueVector,
                         objectCoord=:dirFrame)
)

boxWorldTorque = @instantiateModel(BoxWorldTorque, unitless=true, logCode=false)

stopTime = 5.0
dtmax = 0.1
requiredFinalStates = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -50.03469666280062, -0.13132172058876085, 0.015242310907410017, -17.320661301093185, 10.0, 0.0]
simulate!(boxWorldTorque, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(boxWorldTorque, ["box.rotation", "box.angularVelocity", "torque.torqueVector"], figure=1)

end
