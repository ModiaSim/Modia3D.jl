module Runtests

using  Modia3D
import Modia3D.Test

@enum TestsExtend  shortTests normalTests completeTests
const testsExtend = normalTests
const title = "Modia3D (version=$(Modia3D.Version) with SilentNoPlot)"

@time Test.@testset verbose=true "$title" begin
    usePlotPackage("SilentNoPlot")
    Modia3D.disableRenderer()
    include("includeTests.jl")
    Modia3D.reenableRenderer()
    usePreviousPlotPackage()
end

end
