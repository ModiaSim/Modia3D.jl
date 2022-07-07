module BoxNonLinearSpringDamperPtP

using Modia3D

const interpolatedForceLaws = false
l0 = 0.1
f0 = 5.0
fc(x) = sign(x) * 100.0 * abs(x)^1.2
fd(v) = sign(v) *   2.0 * abs(v)^0.8
if (interpolatedForceLaws)
    using Interpolations
    xc = -1.0:0.1:1.0
    yc = [fc(x) for x in xc]
    fc(x) = CubicSplineInterpolation(xc, yc)(x)
    xd = -10.0:0.1:10.0
    yd = [fd(x) for x in xd]
    fd(v) = LinearInterpolation(xd, yd)(v)
end

SpringDamper = Model3D(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]),
                                   nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(parent=:world, fixedToParent=false,
                   feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    boxCornerFrame = Object3D(parent=:box,
                              feature=Visual(shape=CoordinateSystem(length=:(Length/2))),
                              translation=:[Length/2, Length/2, Length/2]),
    force = SpringDamperPtP(obj1=:world, obj2=:boxCornerFrame, nominalLength=l0, nominalForce=f0, springForceLaw=fc, damperForceLaw=fd)
)

springDamper = @instantiateModel(SpringDamper, unitless=true)

stopTime = 5.0
dtmax = 0.1
if (interpolatedForceLaws)
    requiredFinalStates = [-0.01861362451515635, -0.018612527471268413, -0.2212831534294033, 0.30914769013412663, 0.30913587421631183, 0.019507890185417143, -0.012936800467167768, 0.012897922660753593, 5.844891074857425e-5, -0.24887603798947114, 0.2488565430584932, 8.534690880892765e-7]
else
    requiredFinalStates = [-0.021470308199951636, -0.02147058281625833, -0.22258015450088536, 0.3258869790149358, 0.3258860982698221, 0.007384571696148934, -0.014368985081004778, 0.014360339836395713, 0.00010132676259254706, -0.27207120992766876, 0.27206556875751375, -9.244002396666502e-7]
end
simulate!(springDamper, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingPlotPackage
plot(springDamper, ["box.translation", "box.velocity", "box.rotation", "box.angularVelocity"], figure=1)

end
