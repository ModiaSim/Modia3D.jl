# License for this file: MIT (expat)
# Copyright 2017-2019, DLR Institute of System Dynamics and Control

module Modia3D

const path = dirname(dirname(@__FILE__))   # Absolute path of package directory
const Version = "0.3.0"
const Date = "2019-04-06"

println("\nImporting Modia3D Version $Version ($Date)")



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
abstract type AbstractGravityField      end   # Gravity field type
abstract type AbstractRenderer          end   # Renderer type
abstract type AbstractDLR_VisualizationRenderer <: AbstractRenderer end   # Community or Professional edition of DLR_Visualization renderer
#abstract type AbstractSignal            end   # Signals
#abstract type AbstractBus               end   # Bus: is a collection of signals

abstract type AbstractMassProperties    end

abstract type AbstractKeys    end
abstract type AbstractValues    end

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


# Used renderer (actual value is defined with __init__() below)
const renderer = Vector{AbstractRenderer}(undef,1)


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
using  .Basics
using  .Signals
using  .ForceElements
using  .Solids
using  .Graphics
using  .Composition
import .DLR_Visualization
import .NoRenderer
const Pipe     = Graphics.Pipe        # Pipe cannot be directly exported, due to a conflict with Base.Pipe
const connect  = Composition.connect  # connect cannot be directly exported, due to a conflict with Base.connect


# Called implicitely at the first import/using of Modia3D (when loading Modia3D to the current Julia session)
function __init__()
    info = DLR_Visualization.getSimVisInfo()
    (directory, dll_name, isProfessionalEdition, isNoRenderer) = info

    if isNoRenderer
        renderer[1] = NoRenderer.DummyRenderer(info)
    elseif isProfessionalEdition
        renderer[1] = DLR_Visualization.ProfessionalEdition(info)
    else
        renderer[1] = DLR_Visualization.CommunityEdition(info)
    end
end



export @assembly
export @signal
export @bus
export @forceElement
export Object3D


# Add import clauses used in examples and test
import StaticArrays
import Unitful
import ModiaMath
import LinearAlgebra
import Test

end # module
