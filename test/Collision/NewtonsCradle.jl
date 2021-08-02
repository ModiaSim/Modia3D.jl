"""
    module NewtonsCradle

Model of [Newtons' Cradle](https://en.wikipedia.org/wiki/Newton%27s_cradle).

The balls of the Newton Cradle are initially placed in such a way, that there is a small
distance between the balls, so the balls are **not** touching each other. This distance was
determined (by trial-and-error) in such a way, that collisions between the balls
occur in sequence (so if ball1 hits ball2, then ball3 is hit by ball2 only after the
contact between ball1 and ball2 has already ended; this can be easily determined by
setting `simulate!(..., log=true)`). Only in this case, the simulation matches the
real experiment. If the balls are directly touching each other, multiple contacts
occur at the same time instant and the simulation result is then no longer according
to reality.

See also:

M. Payr (2008): **[An Experimental and Theoretical Study of Perfect Multiple
                Contact Collisions in Linear Chains of Balls.](http://www.zfm.ethz.ch/e/dynamics/payr_collisions.htm)**
                Dissertation, ETH Zürich.
"""
module NewtonsCradle

using ModiaLang
import Modia3D
using  Modia3D.ModiaInterface

vmatVisual = VisualMaterial(color="LightBlue", transparency=0.1)
vmatSolids   = VisualMaterial(color="Red"      , transparency=0.0)

diameter   = 0.02
lengthWire = 0.15
rsmall     = diameter/100

Lx = diameter/2
Ly = 5*diameter
Lz = Lx

Pendulum = Model(
    wire = Object3D(feature=Visual(shape=Cylinder(axis=3, diameter=diameter/5, length=lengthWire),
                                   visualMaterial=vmatVisual)),
    frame1 = Object3D(parent=:wire,
                      translation=[0.0, 0.0, lengthWire/2]),
    frame2 = Object3D(parent=:wire,
                      translation=[0.0, 0.0, -lengthWire/2]),
    cylinder = Object3D(parent=:frame1,
                        feature=Visual(shape=Cylinder(axis=1, diameter=0.6*Lx, length=1.4*Lx),
                                       visualMaterial=vmatSolids)),
    sphere = Object3D(parent=:frame2,
                      feature=Solid(shape=Sphere(diameter=diameter),
                                    solidMaterial="Steel",
                                    visualMaterial=vmatSolids,
                                    contactMaterial="BilliardBall", collision=true)),
)

Cradle = Model(
    gravField = UniformGravityField(g=9.81, n=[0, 0,-1]),
    world = Object3D(feature=Scene(gravityField=:gravField,
                                   nominalLength=Ly,
                                   enableContactDetection=true,
                                   elasticContactReductionFactor=1.0,
                                   animationFile="NewtonsCradle.json",
                                   visualizeBoundingBox=true)),
    box = Object3D(parent=:world,
                   feature=Visual(shape=Box(lengthX=Lx, lengthY=Ly, lengthZ=Lz),
                                  visualMaterial=vmatVisual)),

    frame1 = Object3D(parent=:box, translation=[0.0, -2*(diameter+rsmall), 0.0]),
    frame2 = Object3D(parent=:box, translation=[0.0,   -(diameter+rsmall), 0.0]),
    frame3 = Object3D(parent=:box, translation=[0.0,                  0.0, 0.0]),
    frame4 = Object3D(parent=:box, translation=[0.0,    (diameter+rsmall), 0.0]),
    frame5 = Object3D(parent=:box, translation=[0.0,  2*(diameter+rsmall), 0.0]),

    pendulum1 = Pendulum,
    pendulum2 = Pendulum,
    pendulum3 = Pendulum,
    pendulum4 = Pendulum,
    pendulum5 = Pendulum,

    rev1 = Revolute(obj1=:frame1, obj2=:(pendulum1.frame1), axis=1, canCollide=true, phi=Var(init=-pi/3)),
    rev2 = Revolute(obj1=:frame2, obj2=:(pendulum2.frame1), axis=1, canCollide=true),
    rev3 = Revolute(obj1=:frame3, obj2=:(pendulum3.frame1), axis=1, canCollide=true),
    rev4 = Revolute(obj1=:frame4, obj2=:(pendulum4.frame1), axis=1, canCollide=true),
    rev5 = Revolute(obj1=:frame5, obj2=:(pendulum5.frame1), axis=1, canCollide=true)
)

newtonsCradle = @instantiateModel(buildModia3D(Cradle), unitless=true, log=false, logStateSelection=false, logCode=false)

stopTime = 5.0
tolerance = 1e-8
requiredFinalStates = [-1.180791527568564, -1.033934458288708, 0.04982982739055273, 0.018466029517559813, 0.049844032881367094, 0.018471344490359712, 0.04986127142907837, 0.018476465719447804, 0.04987561416137295, 0.018481494292598495]
simulate!(newtonsCradle, stopTime=stopTime, tolerance=tolerance, log=true, logStates=true, logEvents=true, requiredFinalStates=requiredFinalStates)

@usingModiaPlot
plot(newtonsCradle, ["rev1.phi" "rev5.phi"; "rev1.w" "rev5.w"], figure=1)

end
