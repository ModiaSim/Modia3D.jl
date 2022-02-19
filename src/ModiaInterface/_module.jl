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

export buildModia3D!

import OrderedCollections
import Modia3D
import Modia3D.Composition
using  ModiaLang


# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")
include("$(ModiaLang.path)/models/Translational.jl")

include("buildModia3D.jl")
include("model3D.jl")

end
