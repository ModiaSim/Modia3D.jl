module RuntestsShort

using Modia3D
import Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = shortTests
const title = "Short Modia3D (with SilentNoPlot)"

@time Test.@testset verbose=true "$title" begin
    usePlotPackage("SilentNoPlot")
    Modia3D.disableRenderer()
    include("includeTests.jl")
    Modia3D.reenableRenderer()
    usePreviousPlotPackage()
end

end
