# first idea of implementation
# Very basic example, with signals, only with updatePosition!
# two frames are externaly driven

module Test_BasicExBox2
using Modia3D

vgreen  = Modia3D.Material(color="Green", transparency=0.5)
vyellow = Modia3D.Material(color="Yellow", transparency=0.5)

lx = 1.0
ly = 0.5
lz = 0.3

@enum WaveType sinus cosinus sinush cosinush

@assembly Signals begin
  sinWave = Wave(;wave=sinus)
  cosWave = Wave(;wave=cosinus)
end

mutable struct Wave
  waveType::WaveType
  amplitude::Float64
  frequency::Float64
  phaseShift::Float64
  output
  function Wave(; wave::WaveType=sinus, amplitude::Float64=1.0, frequency::Float64=1/(2*pi), phaseShift::Float64=0.0)
    new(wave,amplitude,frequency,phaseShift)
  end
end

function setSignal!(signals::Signals, time)
  signals.sinWave.output = setOutput(signals.sinWave,time)
  signals.cosWave.output = setOutput(signals.cosWave,time)
end

function setOutput(wave::Wave, time)
  if wave.waveType == sinus
    return wave.amplitude*sin(2*pi*wave.frequency*time + wave.phaseShift)
  elseif wave.waveType == cosinus
    return wave.amplitude*cos(2*pi*wave.frequency*time + wave.phaseShift)
  end
end

@assembly Pendulum(;length=1.0, width=0.2) begin
   world = Modia3D.Object3D()
   green = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vgreen),[ [-lx/2,0,0],[ lx/2,0,0] ])
   rev1 = Modia3D.Revolute(world, green.frames[1])
   gelb = Modia3D.Part(Modia3D.Solid(Modia3D.SolidBox(length,width,width), "Aluminium", vyellow),[ [-lx/2,0,0],[ lx/2,0,0] ])
   rev2 = Modia3D.Revolute(world, gelb.frames[1])
end
pendulum = Pendulum()
waves = Signals()


Modia3D.initAnalysis!(pendulum, Modia3D.SceneOptions(visualizeFrames=true, defaultFrameLength=0.3, enableContactDetection=false))

@static if VERSION >= v"0.7.0-DEV.2005"
    LINSPACE(start,stop,length) = range(0.0, stop=stop, length=length)
else
    LINSPACE(start,stop,length) = linspace(start,stop,length)
end

for time = LINSPACE(0.0, 2*pi, 101)
  setSignal!(waves,time)
  Modia3D.setAngle!(pendulum.rev1, waves.sinWave.output)
  Modia3D.setAngle!(pendulum.rev2, waves.cosWave.output)
  Modia3D.updatePosition!(pendulum)

  Modia3D.visualize!(pendulum,time)
end
Modia3D.closeAnalysis!(pendulum)

println("... success of Test_BasicExBox2.jl!")
end
