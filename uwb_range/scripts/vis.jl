#!/usr/bin/env julia

using Printf
using RobotOS

using StatsPlots, Distributions
using LinearAlgebra
using DelimitedFiles
using Plots
using Statistics

@rosimport mrs_msgs.msg:RangeWithVar
rostypegen()
using .mrs_msgs.msg

data = Dict{String,Vector{Float64}}()

function update_plots()
    plots = []

    for (key, value) in data
        axis = stephist(value, bins=11, normalize=:pdf, fill=true, fillalpha=0.5, label="", color=:red)
        if (size(value)[1] > 2)
            plot!(axis, Normal(mean(value), std(value)), 
            linewidth=3,
            linestyle=:dash,
            color=:black,
            minorgrid=true, 
            title=key, 
            fontfamily="Computer Modern",
            plot_titlefontsize=12,
            label="",
            xguide = "Distance [m]",)
        end
        push!(plots, axis)
    end
    pl = plot(plots...)
    display(pl)
end

function callback(msg::RangeWithVar)
    @printf "Range %.2f m from " msg.range
    @printf "%s\n\r" msg.uav_name

    if !haskey(data, msg.uav_name)
        data[msg.uav_name] = Vector{Float64}()
    end

    append!(data[msg.uav_name], msg.range)
    data[msg.uav_name] = data[msg.uav_name][end-min(100, size(data[msg.uav_name])[1] - 1):end]

    update_plots()
end

function main()
    @printf "Julia visualizer\n\r"

    init_node("rosjl_example")
    sub = Subscriber{RangeWithVar}("/uav/uwb_range/range_out", callback)

    spin()
end

try
    main()
catch err
    @printf "Fail\n"
    rethrow()
end