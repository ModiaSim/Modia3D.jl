module test_massComputation

using  Modia3D
import Modia3D.ModiaMath
using Modia3D.StaticArrays
using Modia3D.Unitful

vmat1 = Modia3D.Material(color="Red", transparency=0.5)
vmat2 = Modia3D.Material(color="Blue", transparency=0.5)

rCM1 = zeros(3)
rCM1[2] = 9.0
rCM2 = zeros(3)
rCM2[1] = 5.0
rCM2[3] = 2.0
I1 = fill(2.0,(3,3))
I2 = fill(0.5,(3,3))
massProp1 = Modia3D.MassProperties(15.0, rCM1, I1)
massProp2 = Modia3D.MassProperties(5.0, rCM2, I2)



rCM3 = zeros(3)
rCM3[2] = 4.0
rCM4 = zeros(3)
rCM4[1] = 3.0
rCM4[3] = 9.0
I3 = fill(20.0,(3,3))
I4 = fill(40.0,(3,3))
massProp3 = Modia3D.MassProperties(60.0, rCM3, I3)
massProp4 = Modia3D.MassProperties(30.0, rCM4, I4)

@signal Sine(;A=1.0, freqHz = 1.0) begin
   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Sine, sim::ModiaMath.SimulationState)
    signal.y.value  = signal.A*sin(2*pi*signal.freqHz*sim.time)
end




@assembly DoublePendulum(;length=1.0, width=0.2) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6))

   part1 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), massProp1, vmat1),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])
   part2 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), massProp2, vmat1),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])

   part3 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), massProp3, vmat2),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])
   part4 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), massProp4, vmat2),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])

   rev1 = Modia3D.Revolute(world          , part1.frames[1])
   Modia3D.connect(part1.frames[2], part2.frames[1]; R=ModiaMath.rot2(45u"°"))
   rev2 = Modia3D.Revolute(part2.frames[2], part3.frame0)
   Modia3D.connect(part3.frames[2], part4.frames[1] ; R=ModiaMath.rot2(-45u"°"))

   sig1 = Sine(A=1.5,freqHz=0.5)
   sig2 = Sine(A=2.5,freqHz=1.0)
   sine1     = Modia3D.SignalToFlangeAngle(sig1.y)
   sine2     = Modia3D.SignalToFlangeAngle(sig1.y)
   Modia3D.connect(sine1,   rev1)
   Modia3D.connect(sine2,   rev2)
end

doublePendulum = DoublePendulum(sceneOptions = Modia3D.SceneOptions(visualizeFrames=true,
                                                                    defaultFrameLength=0.3,
                                                                    enableContactDetection=false))
model = Modia3D.SimulationModel( doublePendulum, analysis=ModiaMath.KinematicAnalysis, useOptimizedStructure = true )
result = ModiaMath.simulate!(model, stopTime=3.0)

ModiaMath.plot(result, ("rev1.phi", "rev2.phi") )


println("... success of test_massComputation.jl!")
end
