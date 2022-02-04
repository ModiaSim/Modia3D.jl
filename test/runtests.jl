module Runtests

using Modia3D
import Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = normalTests
const title = "Modia3D (with SilentNoPlot)"

@time Test.@testset verbose=true "$title" begin
    usePlotPackage("SilentNoPlot")
    Modia3D.disableRenderer()
    include("includeTests.jl")
    Modia3D.reenableRenderer()
    usePreviousPlotPackage()
end

end
