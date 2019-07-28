module Collision_3Elements

using  Modia3D
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="Red", transparency=0.5)
vmat2 = Modia3D.Material(color="Blue", transparency=0.5)
vmat3 = Modia3D.Material(color="Green", transparency=0.5)
cmat  = "Steel"


@signal Sine(;A=1.0, freqHz = 1.0) begin
   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Sine, sim::ModiaMath.SimulationState)
    signal.y.value  = signal.A*sin(2*pi*signal.freqHz*sim.time)
end


@assembly CollisionWithoutJoint(;length=1.0, width=0.2) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6))
   #world_f1 = Modia3D.Object3D(world , r=[0.5, 0.0, 0.5])

   red = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat1;contactMaterial = cmat),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])
   blue = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat2;contactMaterial = cmat),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])
   green = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat3;contactMaterial = cmat),
                        [ [-length/2,0,0] ])

   Modia3D.connect(world, red.frames[1])
   rev2 = Modia3D.Revolute(red.frames[2], blue.frames[1]) #; canCollide = true)
   rev3 = Modia3D.Revolute(blue.frames[2], green.frames[1]) #; canCollide = true)

   sig1 = Sine(A=1.5,freqHz=0.5)
   sig2 = Sine(A=2.5,freqHz=1.0)
   sig3 = Sine(A=3.5,freqHz=1.5)

   sine2     = Modia3D.SignalToFlangeAngle(sig1.y)
   sine3     = Modia3D.SignalToFlangeAngle(sig1.y)

   Modia3D.connect(sine2,   rev2)
   Modia3D.connect(sine3,   rev3)
end


@assembly CollisionWithJoint(;length=1.0, width=0.2) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6))
   world_f1 = Modia3D.Object3D(world , r=[0.5, 0.0, 0.5])

   red = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat1;contactMaterial = cmat),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])
   #blue = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat2;contactMaterial = cmat),
   #                     [ [-length/2,0,0],
   #                       [ length/2,0,0] ])
   blue = Modia3D.Part(Modia3D.Box(length,width,width, material=vmat2),
                                               [ [-length/2,0,0],
                                                 [ length/2,0,0] ])

   green = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat3;contactMaterial = cmat),
                        [ [-length/2,0,0] ])

   rev1 = Modia3D.Revolute(world_f1     , red.frames[1]) #; canCollide = true)
   rev2 = Modia3D.Revolute(red.frames[2], blue.frames[1]) #; canCollide = true)
   rev3 = Modia3D.Revolute(blue.frames[2], green.frames[1]) #; canCollide = true)

   sig1 = Sine(A=1.5,freqHz=0.5)
   sig2 = Sine(A=2.5,freqHz=1.0)
   sig3 = Sine(A=3.5,freqHz=1.5)

   sine1     = Modia3D.SignalToFlangeAngle(sig1.y)
   sine2     = Modia3D.SignalToFlangeAngle(sig1.y)
   sine3     = Modia3D.SignalToFlangeAngle(sig1.y)

   Modia3D.connect(sine1,   rev1)
   Modia3D.connect(sine2,   rev2)
   Modia3D.connect(sine3,   rev3)
end

collWithoutJoint=CollisionWithoutJoint(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.3,enableContactDetection=true))

collWithJoint=CollisionWithJoint(sceneOptions=Modia3D.SceneOptions(visualizeFrames=true,defaultFrameLength=0.3,enableContactDetection=true))


#model = Modia3D.SimulationModel(collWithoutJoint, analysis=ModiaMath.KinematicAnalysis )
model = Modia3D.SimulationModel(collWithJoint, analysis=ModiaMath.KinematicAnalysis )
result = ModiaMath.simulate!(model, stopTime=0.01)

ModiaMath.plot(result, ("rev1.phi", "rev2.phi") )


println("... success of Collision_3Elements.jl!")
end
