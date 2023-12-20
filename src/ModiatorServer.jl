export ModiatorServer

import JSON
import HTTP
import URIParser
using  Base64

# ----------------------------------------------------------------------------------------
function saveJSON(req)
    jsonString = String(req.body);
    index = findfirst("%25NAMEEND%25", jsonString);
    name = jsonString[1:index[1]-1]
    println("Saving ", name)
    jsonString = jsonString[index[end] + 1:end]
    jsonString = URIParser.unescape(jsonString)
    println(jsonString)
    io = open(string(name, ".json"), "w");
    write(io, jsonString);
    close(io)
    response = HTTP.Response(200, "")
    HTTP.setheader(response, "Access-Control-Allow-Origin" => "*")
    return response
end

function loadJSON(req)
    fileURL = String(req.body);
    if (length(fileURL) < 5 || fileURL[end-4:end] != ".json")
        fileURL = string(fileURL, ".json")
    end
    println("Loading JSON file: ", fileURL)
    text = "NOFILEFOUND";
    if (isfile(fileURL))
        obj = open(fileURL, "r");
        text = read(obj, String);
        close(obj)
    end
    response = HTTP.Response(200, text)
    HTTP.setheader(response, "Access-Control-Allow-Origin" => "*")
    return response
end

function saveOBJ(req)
    objString = String(req.body);
    index = findfirst("%25NAMEEND%25", objString);
    name = objString[1:index[1]-1]
    println("Saving ", name)
    objString = objString[index[end] + 1:end]
    objString = URIParser.unescape(objString)
    io = open(string(name, ".obj"), "w");
    write(io, objString);
    close(io)
    response = HTTP.Response(200, "")
    HTTP.setheader(response, "Access-Control-Allow-Origin" => "*")
    return response
end

function loadOBJ(req)
    fileURL = String(req.body);
    println("Loading OBJ file: ", fileURL)
    text = "NOFILEFOUND";
    if (isfile(fileURL))
        stream = open(fileURL, "r");
        text = read(stream, String);
        close(stream)
    end
    response = HTTP.Response(200, text)
    HTTP.setheader(response, "Access-Control-Allow-Origin" => "*")
    return response
end

function loadImageBase64(req)
    fileURL = String(req.body);
    println("Loading Image file: ", fileURL)
    text = "NOFILEFOUND";
    if (isfile(fileURL))
        stream = open(fileURL, "r");
        img = read(stream, String);

        io = IOBuffer();
        iob64_encode = Base64EncodePipe(io);
        write(iob64_encode, img);
        text = String(take!(io));

        close(iob64_encode);
        close(stream);
    end
    response = HTTP.Response(200, text)
    HTTP.setheader(response, "Access-Control-Allow-Origin" => "*")
    return response
end

headers = [
    "Access-Control-Allow-Origin" => "*",
    "Access-Control-Allow-Headers" => "*",
    "Access-Control-Allow-Private-Network" => "true",
    "Access-Control-Allow-Methods" => "POST, GET, OPTIONS"
]

function simulateModel(req)
    # handle CORS requests
    if HTTP.method(req) == "OPTIONS"
        return HTTP.Response(200, headers)
    end
    model = String(req.body);
    model = URIParser.unescape(model);
    experiment = JSON.parse(model);
    @time json = generateSimulationResult(experiment);
#    open("SimRes.txt", "w") do f
#        write(f, json);
#    end
    response = HTTP.Response(200, json)
    HTTP.setheader(response, "Access-Control-Allow-Origin" => "*")
    return response;
end


function generateSimulationResult(experiment)
    println()
    println()
    println("Modia model:")
    println(experiment["modelString"]);
    @time eval(Meta.parse(experiment["modelString"]));
    log = experiment["log"]
    unnamed = @instantiateModel(Unnamed, unitless=true, log=log, logStateSelection=false, logCode=log)
    println("startTime: ", experiment["startTime"], ", stopTime: ", experiment["stopTime"], ", tolerance: ", experiment["tolerance"], ", interval: ", experiment["interval"], " log: ", log);
    simulate!(unnamed, stopTime=experiment["stopTime"], tolerance=experiment["tolerance"], interval=experiment["interval"], log=log, logStates=false, logEvents=false,  useRecursiveFactorizationUptoSize=500) # logTiming=true,
    plotVariables = JSON.parse(experiment["plotVariables"])
    println("Plotting: ", plotVariables)
    if length(plotVariables) > 0
        plot(unnamed, plotVariables, figure=1)
    end
    simulatedResult = JSON.json(get_animationHistory(unnamed, ""));
    println("Simulation finished!");
    return simulatedResult
end

#https://discourse.julialang.org/t/a-simple-http-server-for-showing-files-in-browser/6561/2

#=
function getPublic(req)
    println("getPublic: ")
    request = HTTP.URIs.splitpath(req.target)
    println(request)

    file = request[end] # HTTP.unescapeuri(req.target[end])
    @show file
    return isfile(file) ? HTTP.Response(200, read(file)) : HTTP.Response(404)
end

function getIcon(req)
    println("getIcon: ")
    request = HTTP.URIs.splitpath(req.target)
    println(request)

    file = request[end] # HTTP.unescapeuri(req.target[end])
    @show file
    return true ? HTTP.Response(200, read("C:/Users/hilding.elmqvist/Documents/Modia/ModiaWebApp/motor.svg")) : HTTP.Response(404)
end
=#

function unknownRequest(req)
    println("unknownRequest: ")
    request = HTTP.URIs.splitpath(req.target)
    println(request)
    return HTTP.Response(200, "OK")
end

function ModiatorServer()::Nothing
    ROUTER = HTTP.Router()
    
    HTTP.register!(ROUTER, "POST", "/Modiator/simulateModel/", simulateModel)
    HTTP.register!(ROUTER, "OPTIONS", "/Modiator/simulateModel/", simulateModel)
    
    HTTP.register!(ROUTER, "POST", "/Modiator/saveOBJ/", saveOBJ)
    HTTP.register!(ROUTER, "POST", "/Modiator/loadOBJ/", loadOBJ)
    HTTP.register!(ROUTER, "POST", "/Modiator/loadImageBase64/", loadImageBase64)
    HTTP.register!(ROUTER, "POST", "/Modiator/saveJSON/", saveJSON)
    HTTP.register!(ROUTER, "POST", "/Modiator/loadJSON/", loadJSON)
    #HTTP.register!(ROUTER, "GET", "/Modiator/public/*", getPublic)
    #HTTP.register!(ROUTER, "GET", "/Modiator/icons/*", getIcon)
    HTTP.register!(ROUTER, "GET", "/*", unknownRequest)
    HTTP.register!(ROUTER, "POST", "/*", unknownRequest)
    
    # println("Precompiling")
    # precompile(generateSimulationResult, (Dict,))
    # precompile(HTTP.Router)
    println("Server ready")
    println("Listening to: 127.0.0.1:8000")
    println()
    println("Note that it might take more than 20 seconds for the first simulation to start.")
    
    HTTP.serve(ROUTER, "127.0.0.1", 8000)
    return nothing
end

function julia_main()::Cint
    ModiatorServer()
    return 0
end

