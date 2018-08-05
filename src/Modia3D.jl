# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

module Modia3D
println(" \nWelcome to Modia3D - Modeling and simulation of 3D systems")
println("   Version : 0.2.0-beta.1")

const path = dirname(dirname(@__FILE__))   # Absolute path of Modia3D Julia package directory


# Abstract types
#abstract type AbstractComponentWithVariables end                    # Component that has potentially Variables

abstract type AbstractFlange end              # Flanges are associated with joints, frames, ... (are for connectin parts)

abstract type AbstractObject3Ddata end                                 # Data associated with one Object3D
abstract type AbstractVisualElement  <: AbstractObject3Ddata     end   # Visual element associated with one Object3D
abstract type AbstractGeometry       <: AbstractVisualElement end   # Geometry type
abstract type AbstractCamera         <: AbstractVisualElement end   # Camera type
abstract type AbstractHeadUpDisplay  <: AbstractVisualElement end   # Head-Up-Display type
abstract type AbstractPathDefinition <: AbstractVisualElement end   # Path definition type
abstract type AbstractEffect         <: AbstractVisualElement end   # Effect type

abstract type AbstractSolidGeometry     end   # Immutable shape type that  has a volume and optionally mass. Can be used in collisions.
abstract type AbstractSolidMaterial     end   # Material properties of a solid (e.g. density)
abstract type AbstractContactMaterial   end   # Contact properties of a solid (e.g. spring constant)
abstract type AbstractContactDetection  end   # Contact detection type
abstract type AbstractRenderer          end   # Renderer type
abstract type AbstractGravityField      end   # Gravity field type
#abstract type AbstractSignal            end   # Signals
#abstract type AbstractBus               end   # Bus: is a collection of signals

import ModiaMath
abstract type AbstractAssemblyInternal  <: ModiaMath.AbstractComponentInternal end
abstract type AbstractAssemblyComponent <: ModiaMath.AbstractComponentWithVariables end # Objects that can be stored in an Assembly (has elements name and within)
abstract type AbstractAssembly          <: AbstractAssemblyComponent end  # Assembly object (has elements name, within, referenceObject3D and scene)
abstract type AbstractTwoObject3DObject <: AbstractAssemblyComponent end  # Object related to two Object3Ds
abstract type AbstractJoint             <: AbstractTwoObject3DObject end  # Constraint between two Object3Ds
abstract type AbstractRevolute          <: AbstractJoint             end  # Revolute joint
abstract type AbstractPrismatic         <: AbstractJoint             end  # Prismatic joint

abstract type AbstractForceTorque <: AbstractAssemblyComponent end  # Object computing the torque of a revolute joint

abstract type AbstractSignal <: AbstractAssemblyComponent end # It is a Signal with one output variable
abstract type AbstractSignalAdaptor  <: AbstractAssemblyComponent end
abstract type AbstractForceAdaptor   <: AbstractAssemblyComponent end


# Enumerations
@enum Ternary      True False Inherited
@enum AnalysisType         KinematicAnalysis QuasiStaticAnalysis           DynamicAnalysis
@enum VariableAnalysisType AllAnalysis       QuasiStaticAndDynamicAnalysis OnlyDynamicAnalysis NotUsedInAnalysis


# Include sub-modules
include(joinpath("Basics"          , "_module.jl"))
include(joinpath("Signals"         , "_module.jl"))
include(joinpath("ForceElements"   , "_module.jl"))
include(joinpath("Graphics"        , "_module.jl"))
include(joinpath("Solids"          , "_module.jl"))
include(joinpath("Composition"     , "_module.jl"))
include(joinpath("renderer"        , "DLR_Visualization"  , "_module.jl"))
include(joinpath("renderer"        , "NoRenderer"         , "_module.jl"))
include(joinpath("contactDetection", "ContactDetectionMPR", "_module.jl"))


# Make symbols available that have been exported in sub-modules
using .Basics
using .Signals
using .ForceElements
using .Solids
using .Graphics
using .Composition
const Pipe     = Graphics.Pipe        # Pipe cannot be directly exported, due to a conflict with Base.Pipe
const connect  = Composition.connect  # connect cannot be directly exported, due to a conflict with Base.connect


export @assembly
export @signal
export @bus
export @forceElement
export Object3D


end # module
