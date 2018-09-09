# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


mutable struct AssemblyInternal <: ModiaMath.AbstractComponentInternal
   name::Symbol                                   # Instance name of assembly within parent assembly (is set by parent assembly)
   within::Union{Modia3D.AbstractAssembly,NOTHING}   # Parent assembly (if within==nothing, there is no parent assembly yet)
   scene::Union{Scene,NOTHING}                       # Scene in which the Assembly is present (if scene==nothing, Assembly is not yet in a scene)
   sceneOptions::Union{SceneOptions,NOTHING}         # SceneOptions defined for the Assembly (if sceneOptions==nothing, no sceneOptions are defined)
   referenceObject3D::Union{Object3D,NOTHING}        # Reference frame of assembly (if referenceObject3D==nothing, Assembly has not yet a reference frame)

   uniqueSignals::Array{Modia3D.AbstractSignal}
   uniqueForceTorques::Array{Modia3D.AbstractForceTorque}
   potentialVarInput::Array{ModiaMath.RealScalar}
   potentialVarOutput::Array{ModiaMath.RealScalar}
   flowVarInput::Array{ModiaMath.RealScalar}
   flowVarOutput::Array{ModiaMath.RealScalar}
   AssemblyInternal(name,sceneOptions=nothing) = new(Symbol(name), nothing, nothing, sceneOptions, nothing,
                    Array{Modia3D.AbstractSignal,1}(),Array{Modia3D.AbstractForceTorque,1}(),Array{ModiaMath.RealScalar,1}(),Array{ModiaMath.RealScalar,1}(),Array{ModiaMath.RealScalar,1}(),Array{ModiaMath.RealScalar,1}())
end


function initAssemblyComponent!(within::Modia3D.AbstractAssembly, component::Any, name)::NOTHING
    setfield!(within, Symbol(name), component)
    return nothing
end

function initAssemblyComponent!(within::Modia3D.AbstractAssembly, component::Modia3D.AbstractAssemblyComponent, name)::NOTHING
   component._internal.within = within
   component._internal.name   = Symbol(name)

   # within.name = component
   setfield!(within, Symbol(name), component)
   return nothing
end

function initAssemblyComponent!(within::Modia3D.AbstractAssembly, component::Object3D, name)::NOTHING
   component._internal.within = within
   component._internal.name   = Symbol(name)

   if component === component.parent && typeof(within._internal.referenceObject3D) == NOTHING
      within._internal.referenceObject3D = component
   end

   # within.name = component
   setfield!(within, Symbol(name), component)
   return nothing
end

function initAssemblyComponent!(within::Modia3D.AbstractAssembly, component::AbstractVector, name)::NOTHING
   for i in eachindex(component)
      component[i]._internal.within = within
      component[i]._internal.name   = Symbol( string(name) * "[" * string(i) * "]" )
   end

   # within.name = component
   setfield!(within, Symbol(name), component)
   return nothing
end



#-------------------------  Print assemblies -----------------------------
function Base.show(io::IO, a::Modia3D.AbstractAssemblyComponent)
   #if typeof(a._internal.scene) == NOTHING
      print(io, typeof(a), "(\n")
   #else
   #   print(io, typeof(a), "(scene=[...],")
   #end

   for c in fieldnames(typeof(a))
      field = getfield(a,c)
      if typeof(field) <: Modia3D.AbstractAssembly
         println(io,"\n   ", c, " = ", field)
      elseif typeof(field) <: AbstractVector
         for i in 1:length(field)
            println(io,"   ", c, "[", i, "] = ", field[i])
         end
      elseif typeof(field) != AssemblyInternal
         # Print fields, but not "_internal::Modia3D.Assembly"
         println(io,"   ", c, " = ", field)
      end
   end
   print(io, "   )")
end


#=
"""    printObject3DAndAllChildren(frame) - print frame and all of its children"""
function printObject3DAndAllChildren(frame::Object3D)::NOTHING
   twoObject3Dobjects = ObjectIdDict()
   if hasParent(frame)
      stack = frame.scene.stack
      empty!(stack)
   else
      stack = Object3D[]
   end
   push!(stack, frame)

   while length(stack) > 0
      frame1 = pop!(stack)
      println(frame1)
      for i in length(frame1.children):-1:1
         push!(stack, frame1.children[i])
      end
      for obj in frame1.twoObject3Dobject
         twoObject3Dobjects[obj] = obj
      end
   end

   if length(twoObject3Dobjects) > 0
      println("TwoObject3DObjects = [")
      first = true
      for (key,value) in twoObject3Dobjects
         if first
            first = false
         else
            println(",")
         end
         println("  ", value)
      end
      println("]")
   end

   return nothing
end


function JSON.show_json(io::JSON.StructuralContext,
                         s::JSON.CommonSerialization, scene::Scene)
   dictJoints = ObjectIdDict()
   stackObj   = Modia3D.AbstractObject3Ddata[]

   JSON.begin_object(io)
      JSON.show_pair(io, s, "name", scene.name)

      # Store all Object3Ds in sequence
      stack = scene.stack
      push!(stack, scene.world)
      JSON.show_key(io, "frames")
      JSON.begin_object(io)
         while length(stack) > 0
            frame = pop!(stack)
            JSON.show_key(io, frame.name)
            JSON.show_json(io, s, frame)

            # Put children on stack
            append!(stack, frame.children)

            # Put joint on dictJoints, if available
            if hasJoint(frame)
               dictJoints[frame.joint] = frame.joint
            end

            # Put object3D on stackObj, if available
            if hasData(frame) && typeof(frame.data) != Scene
               push!(stackObj, frame.data)
            end

            # Put twoObject3Dobject joints on dictJoints, if available
            for obj in frame.twoObject3Dobject
               if typeof(obj) <: Modia3D.AbstractJoint
                  dictJoints[obj] = obj
               end
            end
         end
      JSON.end_object(io)

      # Store all objects in sequence (sorted by name)
      perm = sortperm( [obj.name for obj in stackObj] )
      JSON.show_key(io, "objects3D")
      JSON.begin_object(io)
         for obj in stackObj[perm]
            JSON.show_key(io, obj.name)
            JSON.show_json(io, s, obj)
         end
      JSON.end_object(io)

      # Store all joints in sequence (sorted by name)
      JSON.show_key(io, "joints")
      joints = collect(keys(dictJoints))
      perm   = sortperm( [j.name for j in joints] )
      JSON.begin_object(io)
         for joint in joints[perm]
            JSON.show_key(io, joint.name)
            JSON.show_json(io, s, joint)
         end
      JSON.end_object(io)
   JSON.end_object(io)
end


function JSON.show_json(io::JSON.StructuralContext,
                         s::JSON.CommonSerialization, frame::Object3D)
   JSON.begin_object(io)
      JSON.show_pair(io, s, "scene"      , inScene(frame)   ? frame.scene.name : nothing)
      JSON.show_pair(io, s, "parent"     , hasParent(frame) ? frame.parent.name : nothing)
      JSON.show_pair(io, s, "children"   , [child.name for child in frame.children] )
      JSON.show_pair(io, s, "joint"      , hasJoint(frame) ? frame.joint.name : nothing)
      if frame.r_rel != ModiaMath.ZeroVector3D
         JSON.show_pair(io, s, "r_rel", frame.r_rel)
      end
      if frame.R_rel != ModiaMath.NullRotation
         JSON.show_pair(io, s, "R_rel", frame.R_rel)
      end
      if frame.r_abs != ModiaMath.ZeroVector3D
         JSON.show_pair(io, s, "r_abs", frame.r_abs)
      end
      if frame.R_abs != ModiaMath.NullRotation
         JSON.show_pair(io, s, "R_abs", frame.R_abs)
      end

      JSON.show_pair(io, s, "object3D"      , hasData(frame) ? frame.data.name : nothing)
      JSON.show_pair(io, s, "twoObject3Dobject", [obj.name for obj in frame.twoObject3Dobject])
   JSON.end_object(io)
end


"""
    writeObject3DAndAllChildrenOnJsonFile(frame; file=frame.name * ".json")

Write frame and all of its children in JSON format on file
"""
function writeObject3DAndAllChildrenOnJsonFile(frame::Object3D; file = frame.name*".json")::NOTHING
   if notInScene(frame)
      error("\nError from writeObject3DAndAllChildrenOnJsonFile:\n",
            "Can currently only write frames on file that are in a scene, but ", frame.name, " is not.")
   end
   open(file, "w") do io
      JSON.print(io, frame.scene, 3)
   end
end

=#


using Base.Meta:quot, isexpr

"""
    @assembly AssemblyName(arguments) begin ... end

Return the constructor for a new struct `AssemblyName`
consisting of Object3Ds that are connected together.
The new struct consists of all left-hand-side (scalar or vector) symbols
present between `begin ... end`.

# Examples
```julia
using Modia3D

@assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly) begin
   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vmat1))
   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])
   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])
   cyl1   = Modia3D.Object3D(frame1, cyl)
   cyl2   = Modia3D.Object3D(frame2, cyl)
end
bar = Bar(;Lx=1.0)
Modia3D.visualizeAssembly!( bar )
```
"""
macro assembly(head, top_ex)
   # @assembly Name(arguments) begin ... end
   #    is mapped to:
   # mutable struct Name <: Modia3D.AbstractAssembly
   #    < declarations >
   #    function Name(sceneOptions=nothing; <arguments>)
   #       < equations top_ex >
   #       _internal = Modia3D.AssemblyInternal(:Name, sceneOptions)
   #       a = new(_internal, <declarations>)
   #       Modia3D.initAssemblyComponent(_internal, name1, :name1)
   #         ...
   #       return a
   #    end
   # end
   #
   # Todo: use  "gensym()" instead of "a" for the
   #
   # names:
   #
   if typeof(head) == Symbol
      assembly_symbol = head
      fc = Expr(:call, head)
   else
      @assert isexpr(head, :call)
      assembly_symbol = head.args[1]::Symbol
      fc = head
   end


   # Build:  internalAssemblyName = Modia3D.Assembly(assembly_name)
   # internalAssemblyName = gensym()
   # push!(eq, :( a = Modia3D.Assembly($assembly_name)) )

   # Inspect top_ex find
   #   symbol = < statements >
   # and store "symbol" in "names"
   this = gensym("this")
   eqInternal = []
   eq = []
   push!(eqInternal,:( $this = new( Modia3D.Composition.AssemblyInternal($(quot(assembly_symbol)),sceneOptions) ) ))
   namesSet = DataStructures.OrderedSet{Symbol}() # Set of left-hand side symbols
   for ex in top_ex.args
     # dump(ex)
      push!(eq, ex)
      if isexpr(ex, :(=), 2) && typeof(ex.args[1]) == Symbol
         symbol = ex.args[1]
         if in(symbol, namesSet)
            error("\nfrom @assembly ", assembly_symbol, ": Left hand side symbol in",
                  "\n   ", ex,
                  "\nis defined twice.")
         end
         push!(namesSet, symbol)
         push!(eq, :( Modia3D.Composition.initAssemblyComponent!($this,$symbol,$(quot(symbol)) ) ) )

 #        str = String(symbol)
 #        push!(eq, :( Modia3D.Composition.initAssemblyComponent!($this,$symbol,$str) ) )
     end
   end
   names = collect(namesSet)

   #helpArray = []
   #for name in names
   #  push!(helpArray, :(me.$name = $name))
   #end

   # Build:
   #   mutable struct assembly_symbol <: Modia3D.AbstractAssembly
   #      _internal::Modia3D.AssemblyInternal
   #      names[1]
   #      names[2]
   #       ...
   #      function assembly_symbol(...)
   #         $(eq...)
   #         Modia3D.Composition.initAssemblyInternal!( new(...) )
   #      end
   #   end
   code = :(mutable struct $assembly_symbol <: Modia3D.AbstractAssembly; end)
   code.args[3].args = vcat( :( _internal::Modia3D.Composition.AssemblyInternal ), names )

   code2 = code.args[3].args
   push!(code2, :(function fc(); end) )
   i = length(code2)
   code2[i].args[1] = fc

   # Add keyword argument sceneOptions to the arguments of the constructor function
   fcp = code2[i].args[1]
   if length(fcp.args) <= 1
      # No arguments; add new keyword argument sceneOptions
      push!(fcp.args, Expr(:parameters, Expr(:kw, :sceneOptions, :nothing) ))
   elseif typeof(fc.args[2]) == Symbol
      # Positional arguments present, but no keyword arguments yet; insert new kewword argument
      insert!(fcp.args, 2, Expr(:parameters, Expr(:kw, :sceneOptions, :nothing) ))
   elseif fc.args[2].head == :parameters
      # Keyword arguments already present; add sceneOptions
      push!(fcp.args[2].args, Expr(:kw, :sceneOptions, :nothing))
   else
      error("Unknown error in assembly.jl: typeof(fc.args[2]) = ", typeof(fc.args[2]))
   end

   # Generate constructor function
   code2[i].args[2] = quote
      $(eqInternal...)
 #     me = new($this)
      $(eq...)
#      $(helpArray...)
      return $this
   end

   # Print generated function (without "#....\n")
   # println( replace(sprint(showcompact,code) , r"# .*\n", "\n") )
   # println( sprint(showcompact,code) )

   return esc(code)
end
