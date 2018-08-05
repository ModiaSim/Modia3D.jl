# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   ModiaMath.Variables (ModiaMath/Variables/_module.jl)
#

using Base.Meta:quot, isexpr

"""
    @signal signalName(arguments) begin ... end - Generate a function that instantiates a new signal

`signalName(arguments) begin ... end` is treated as the constructor function of a new
(mutable) struct that consists of all left-hand-side (scalar or vector) symbols
present between `begin ... end`.

# Examples
```julia
using ModiaMath

```
"""
macro signal(head, top_ex)
   # @signal Name(arguments) begin ... end
   #    is mapped to:
   # mutable struct Name <: ModiaMath.AbstractComponentWithVariables
   #    < declarations >
   #    function Name(<arguments>)
   #       < equations top_ex >
   #       _internal  = ModiaMath.ComponentInternal(:Name, nothing, computeVariables!)
   #       _signal = new(_internal, <declarations>)
   #       ModiaMath.initComponent(_internal, name1, :name1)
   #       ModiaMath.initComponent(_internal, name2, :name2)
   #         ...
   #       return _signal
   #    end
   # end

   if typeof(head) == Symbol
      signal_symbol = head
      fc = Expr(:call, head)
   else
      @assert isexpr(head, :call)
      signal_symbol = head.args[1]::Symbol
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
   push!(eqInternal,:( $this = new( ModiaMath.ComponentInternal($(quot(signal_symbol))) ) ))
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
            error("\nfrom @signal ", signal_symbol, ": keyword argument",
                  "\n   ", kw,
                  "\nis defined twice.")
         end
         push!(namesSet, symbol)
         push!(helpArray, :($this.$symbol = $symbol))  
      end
      # push!(fcp.args[2].args, Expr(:kw, :sceneOptions, :nothing))
   else
      error("Unknown error in signals.jl: typeof(fc.args[2]) = ", typeof(fc.args[2]))
   end

   for ex in top_ex.args
      # dump(ex)
      push!(eq, ex)
      if isexpr(ex, :(=), 2) && typeof(ex.args[1]) == Symbol
         symbol = ex.args[1]
         if in(symbol, namesSet)
            error("\nfrom @signal ", signal_symbol, ": Left hand side symbol in",
                  "\n   ", ex,
                  "\nis defined twice.")
         end
         push!(namesSet, symbol)
         push!(eq, :( ModiaMath.initComponent!($this,$symbol,$(quot(symbol)) ) ) )
     end
   end
   names = collect(namesSet)


   # Build:
   #   mutable struct signal_symbol <: ModiaMath.AbstractComponentWithVariables
   #      _internal::ModiaMath.ComponentInternal
   #      names[1]
   #      names[2]
   #       ...
   #      function signal_symbol(...)
   #         $(eq...)
   #         ModiaMath.initComponentInternal!( new(...) )
   #      end
   #   end
   code = :(mutable struct $signal_symbol <: Modia3D.AbstractSignal; end)
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
