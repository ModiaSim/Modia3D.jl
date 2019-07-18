module Simulate_MassWithDamper

using  Modia3D
import Modia3D.ModiaMath

# Solids
groundMaterial = Modia3D.Material(color="DarkGreen", transparency=0.5)
vmat1 = Modia3D.Material(color="LightBlue" , transparency=0.5)
vmat2 = Modia3D.Material(color="Red")


# Damper
@forceElement PDamper(; d=1.0) begin
    v = ModiaMath.RealScalar("v", causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    f = ModiaMath.RealScalar("f", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end
function Modia3D.computeTorque(damper::PDamper, sim::ModiaMath.SimulationState)
    damper.f.value = -damper.d*damper.v.value
end


@assembly MassWithDamper(;Lx = 0.1, Ly=0.1, Lz=0.1, m=1.0) begin
   world  = Object3D(visualizeFrame=true)

   body   = Object3D(Solid(SolidBox(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   prismatic = Prismatic(world, body, axis=2, v_start=0.1)

   damper         = PDamper(d=0.2)
   damper_adaptor = Modia3D.AdaptorForceElementToPFlange(v=damper.v, f=damper.f)
   Modia3D.connect(damper_adaptor, prismatic)
end

gravField      = UniformGravityField(g=9.81, n=[0,0,-1])
massWithDamper = MassWithDamper(sceneOptions=SceneOptions(gravityField=gravField, visualizeFrames=true, defaultFrameLength=0.3))
model  = Modia3D.SimulationModel( massWithDamper )
result = ModiaMath.simulate!(model, stopTime=5.0, tolerance=1e-4, log=true)

ModiaMath.plot(result, ["prismatic.s", "prismatic.v", "prismatic.a", "prismatic.f"])

println("... success of Simulate_MassWithDamper.jl!")
end
