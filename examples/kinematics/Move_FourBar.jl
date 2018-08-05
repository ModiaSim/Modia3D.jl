module Move_FourBar

import ModiaMath
using Modia3D


# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
massProperties = "Aluminium"
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")

@assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2)
   cyl1   = Modia3D.Object3D(frame1, cyl)
   cyl2   = Modia3D.Object3D(frame2, cyl)
end

@assembly Fourbar(;Lx = 0.1, Ly=Lx/5, Lz=Ly, groundWidth=Lx, groundHeight=0.1*Lx) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.6))
   frame1 = Modia3D.Object3D(world ; r=[0.5*Lx, 0.0, groundWidth/2])
   frame2 = Modia3D.Object3D(frame1; r=[Lx    , 0.0, 0.0])
   ground = Modia3D.Object3D(world , Modia3D.Box(3*Lx, groundWidth, groundHeight; material=groundMaterial);
                                              r = [1.5*Lx, -groundHeight/2, groundWidth/2],
                                              R = ModiaMath.rot_nxy([-1,0,0], [0,0,1]))
   bar1   = Bar(Lx=Lx, Ly=Ly, Lz=Lz)
   bar2   = Bar(Lx=Lx, Ly=Ly, Lz=Lz)
   bar3   = Bar(Lx=Lx, Ly=Ly, Lz=Lz)

   rev1   = Modia3D.Revolute(frame1     , bar1.frame1; phi_start =  pi/2)
   rev2   = Modia3D.Revolute(bar1.frame2, bar2.frame1; phi_start = -pi/2)
   rev3   = Modia3D.Revolute(frame2     , bar3.frame1; phi_start =  pi/2)
   rev4   = Modia3D.Revolute(bar3.frame2, bar2.frame2; phi_start = NaN)          # phi_start = -pi/2
end


#@signal LinearMovement(;yStart = 0.0, yDelta = 1.0) begin
#   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
#end
#function Modia3D.computeSignal(signal::LinearMovement, sim::ModiaMath.SimulationState)
#    signal.y.value = signal.yStart + signal.yDelta*(sim.time-sim.startTime)/(sim.stopTime-sim.startTime)
#end

@signal Sine(;yStart = 0.0, A = 1.0, freqHz = 1.0) begin
   y = ModiaMath.RealScalar(causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeSignal(signal::Sine, sim::ModiaMath.SimulationState)
    signal.y.value = signal.yStart + signal.A*sin(2*pi*signal.freqHz*sim.time)
end


Lx = 1.0
@assembly Move begin
   fourbar = Fourbar(Lx=Lx)

   sine = Sine(yStart=fourbar.rev1.phi.start, A=-pi/3, freqHz=0.5)
   sig  = Modia3D.SignalToFlangeAngle(sine.y)
   Modia3D.connect(sig, fourbar.rev1)
end

model  = Modia3D.SimulationModel( Move(), analysis=ModiaMath.KinematicAnalysis )
result = ModiaMath.simulate!(model, stopTime=3.0, log=true)

println("... success of Move_FourBar.jl!")
end
