# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ForceElements (Modia3D/ForceElements/_module.jl)
#

using Base.Meta:quot, isexpr

"""
    @forceElement forceName(arguments) begin ... end - Generate a function that instantiates a new forceElement

`forceName(arguments) begin ... end` is treated as the constructor function of a new
(mutable) struct that consists of all left-hand-side (scalar or vector) symbols
present between `begin ... end`.

# Examples

```
julia> include("$(Modia3D.path)/examples/dynamics/Simulate_PendulumWithDamper.jl");
```

```julia
using ModiaMath
using Modia3D

@forceElement Damper(; d=1.0) begin
    w   = ModiaMath.RealScalar("w",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)
    tau = ModiaMath.RealScalar("tau", causality=ModiaMath.Output, numericType=ModiaMath.WR)
end;
function Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)
    damper.tau.value = -damper.d*damper.w.value
end;
@assembly PendulumWithDamper(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0, g=9.81) begin
   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))
   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))
   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])
   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))

   # Connect pendulum to world with a damper in the joint
   rev    = Modia3D.Revolute(world, frame1)
   d      = Damper(d=0.2)
   damper = Modia3D.AdaptorForceElementToFlange(w=d.w, tau=d.tau)
   Modia3D.connect(damper, rev)
end
pendulum = PendulumWithDamper()
model = Modia3D.SimulationModel( pendulum )
result = ModiaMath.simulate!(model, stopTime=5.0, interval=0.1, tolerance=1e-4, log=true)
ModiaMath.plot(result, ["rev.phi", "rev.w", "rev.a", "rev.tau"])
```
"""
macro forceElement(head, top_ex)
   # @forceElement Name(arguments) begin ... end
   #    is mapped to:
   # mutable struct Name <: ModiaMath.AbstractComponentWithVariables
   #    < declarations >
   #    function Name(<arguments>)
   #       < equations top_ex >
   #       _internal  = ModiaMath.ComponentInternal(:Name, nothing, computeVariables!)
   #       _force = new(_internal, <declarations>)
   #       ModiaMath.initComponent(_internal, name1, :name1)
   #       ModiaMath.initComponent(_internal, name2, :name2)
   #         ...
   #       return _force
   #    end
   # end

   if typeof(head) == Symbol
      force_symbol = head
      fc = Expr(:call, head)
   else
      @assert isexpr(head, :call)
      force_symbol = head.args[1]::Symbol
      fc = head
   end


   # Build:  internalComponentName = ModiaMath.Component(component_name)
   # internalComponentName = gensym()
   # push!(eq, :( a = ModiaMath.Component($component_name)) )

   # Inspect top_ex find
   #   symbol = < statements >
   # and store "symbol" in "names"
   this = gensym("this")
   eqInternal = []
   eq      = []
   push!(eqInternal,:( $this = new( ModiaMath.ComponentInternal($(quot(force_symbol))) ) ))
   namesSet = DataStructures.OrderedSet{Symbol}() # Set of left-hand side symbols
   helpArray = []

   # Add keyword arguments to namesSet
   if length(fc.args) <= 1
      # No keyword arguments
   elseif typeof(fc.args[2]) == Symbol
      # Positional arguments present, but no keyword arguments
   elseif fc.args[2].head == :parameters
      # Keyword arguments present
      kwargs = fc.args[2].args
      # println("... keyword arguments = ", kwargs)
      for kw in kwargs
         # println("... kw = ", kw, ", kw.head = ", kw.head, ", kw.args = ", kw.args, ", name = ", kw.args[1])
         symbol = kw.args[1]
         if in(symbol, namesSet)
            error("\nfrom @forceElement ", force_symbol, ": keyword argument",
                  "\n   ", kw,
                  "\nis defined twice.")
         end
         push!(namesSet, symbol)
         push!(helpArray, :($this.$symbol = $symbol))
      end
      # push!(fcp.args[2].args, Expr(:kw, :sceneOptions, :nothing))
   else
      error("Unknown error in forces.jl: typeof(fc.args[2]) = ", typeof(fc.args[2]))
   end

   for ex in top_ex.args
      # dump(ex)
      push!(eq, ex)
      if isexpr(ex, :(=), 2) && typeof(ex.args[1]) == Symbol
         symbol = ex.args[1]
         if in(symbol, namesSet)
            error("\nfrom @forceElement ", force_symbol, ": Left hand side symbol in",
                  "\n   ", ex,
                  "\nis defined twice.")
         end
         push!(namesSet, symbol)
         push!(eq, :( ModiaMath.initComponent!($this,$symbol,$(quot(symbol)) ) ) )
     end
   end
   names = collect(namesSet)

   # Build:
   #   mutable struct force_symbol <: ModiaMath.AbstractComponentWithVariables
   #      _internal::ModiaMath.ComponentInternal
   #      names[1]
   #      names[2]
   #       ...
   #      function force_symbol(...)
   #         $(eq...)
   #         ModiaMath.initComponentInternal!( new(...) )
   #      end
   #   end
   code = :(mutable struct $force_symbol <: Modia3D.AbstractForceTorque; end)
   # println("code.args = ", code.args)
   code.args[3].args = vcat( :( _internal::ModiaMath.ComponentInternal ), names )

   code2 = code.args[3].args
   push!(code2, :(function fc(); end) )
   i = length(code2)
   code2[i].args[1] = fc

   # Generate constructor function
   code2[i].args[2] = quote
      $(eqInternal...)
      $(helpArray...)
      $(eq...)
      return $this
   end

   # Print generated function (without "#....\n")
   # println( replace(sprint(showcompact,code) , r"# .*\n", "\n") )
   # println( sprint(showcompact,bcode) )

   return esc(code)
end
