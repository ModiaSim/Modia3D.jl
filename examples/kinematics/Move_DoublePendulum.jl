module Move_DoublePendulum

using  Modia3D
import Modia3D.ModiaMath

vmat1 = Modia3D.Material(color="Red", transparency=0.5)
vmat2 = Modia3D.Material(color="Blue", transparency=0.5)

@signal Sine(;A=1.0, freqHz = 1.0) begin
   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Sine, sim::ModiaMath.SimulationState)
    signal.y.value  = signal.A*sin(2*pi*signal.freqHz*sim.time)
end


@assembly DoublePendulum(;length=1.0, width=0.2) begin
   world = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6))

   part1 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat1),
                        [ [-length/2,0,0],
                          [ length/2,0,0] ])
   part2 = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vmat2),
                        [ [-length/2,0,0] ])

   rev1 = Modia3D.Revolute(world          , part1.frames[1])
   rev2 = Modia3D.Revolute(part1.frames[2], part2.frames[1])

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


println("... success of Move_DoublePendulum.jl!")
end
