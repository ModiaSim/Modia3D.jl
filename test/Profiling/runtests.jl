module ProfilingTests

# Running this file requires to set 
#    const useTimer = true
# at the beginning of file Modia3D/src/Composition/_module.jl


import Test
using  ModiaLang

@time Test.@testset verbose=true "ProfilingTests" begin
    usePlotPackage("SilentNoPlot")

    include("BouncingSphere_with_time.jl")
    include("Mobile.jl")
    
    usePreviousPlotPackage()
end


end
