module ModiaInterface

export Model3D
export Object3D, Scene, Visual, Solid
export Box, Beam, Cylinder, Sphere, Ellipsoid
export Cone, Capsule, GearWheel, Grid, SpringShape
export CoordinateSystem, FileMesh, ModelicaShape
export Font, TextShape
export VisualMaterial
export MassProperties, MassPropertiesFromShape
export MassPropertiesFromShapeAndMass
export UniformGravityField
export RefPath, ptpJointSpace, scheduleReferenceMotion
export calculateRobotMovement
export getRefPathPosition, getRefPathInitPosition, getVariables
export Fix
export Revolute, RevoluteWithFlange
export Prismatic, PrismaticWithFlange
export J123, J132, J123or132, singularRem, FreeMotion, change_rotSequenceInNextIteration!
export Bushing, SpringDamperPtP

export buildModel3D!, get_animationHistory

import OrderedCollections
import Modia3D
import Modia3D.Composition
using  Modia
using  StaticArrays


# Modia models
include("$(Modia.path)/models/Blocks.jl")
include("$(Modia.path)/models/Electric.jl")
include("$(Modia.path)/models/Rotational.jl")
include("$(Modia.path)/models/Translational.jl")

include("buildModel3D.jl")
include("model3D.jl")

end
