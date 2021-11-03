module BoxNonLinearSpringDamperPtP

using ModiaLang

import Modia3D
using  Modia3D.ModiaInterface

interpolatedForceLaws = false
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

SpringDamper = Model(
    Length = 0.1,
    Mass = 1.0,
    IMoment = 0.1,
    visualMaterial = VisualMaterial(color="IndianRed1", transparency=0.5),
    world = Object3D(feature=Scene(gravityField=UniformGravityField(g=9.81, n=[0, 0, -1]), nominalLength=:Length)),
    worldFrame = Object3D(parent=:world,
                          feature=Visual(shape=CoordinateSystem(length=:Length))),
    box = Object3D(feature=Solid(shape=Box(lengthX=:Length, lengthY=:Length, lengthZ=:Length),
                                 massProperties=MassProperties(; mass=:Mass, Ixx=:IMoment, Iyy=:IMoment, Izz=:IMoment),
                                 visualMaterial=:(visualMaterial))),
    boxCornerFrame = Object3D(parent=:box,
                              feature=Visual(shape=CoordinateSystem(length=:(Length/2))),
                              translation=:[Length/2, Length/2, Length/2]),
    joint = FreeMotion(obj1=:world, obj2=:box),
    force = SpringDamperPtP(obj1=:world, obj2=:boxCornerFrame, nominalLength=l0, nominalForce=f0, springForceLaw=fc, damperForceLaw=fd)
)

springDamper = @instantiateModel(buildModia3D(SpringDamper), aliasReduction=false, unitless=true)

stopTime = 5.0
dtmax = 0.1
if (interpolatedForceLaws)
    requiredFinalStates = [-0.018609840194419268, -0.018610063010666804, -0.22128548799442815, 0.3091309586280587, 0.3091307182143652, 0.019491580480008697, -0.01289771100508473, 0.012897263597266288, 8.547830805609731e-5, -0.24873244248570697, 0.2487317559006742, 6.865934996471178e-7]
else
    requiredFinalStates = [-0.021471022585945587, -0.021471338063312084, -0.22257979374805176, 0.3258966598257503, 0.3258956068893249, 0.007390650203910464, -0.014365742162847027, 0.014364567804398353, 0.00010407183716342378, -0.2720750966146867, 0.27207423475429426, 8.60986118342952e-7]
end
simulate!(springDamper, stopTime=stopTime, dtmax=dtmax, log=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(springDamper, ["joint.r", "joint.v", "joint.rot", "joint.w"], figure=1)

end
