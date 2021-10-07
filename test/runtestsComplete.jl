module RuntestsComplete

using  ModiaLang
import Modia3D
import Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = completeTests
const title = "Complete Modia3D (with SilentNoPlot)"

@time Test.@testset verbose=true "$title" begin
    usePlotPackage("SilentNoPlot")
    Modia3D.disableRenderer()
    include("includeTests.jl")
    Modia3D.reenableRenderer()
    usePreviousPlotPackage()
end

end
