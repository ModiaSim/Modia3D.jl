module test_Examples_Collision_Billard

import Modia3D

# test/collision/Billard_Simulations
collisionPath = joinpath(Modia3D.path, "test", "collision","Billard_Simulations")

include(joinpath(collisionPath, "collision_Billard_Prismatic.jl")) # takes long time


println("\n ...test_Examples_Collision_Billard finished!")
end
