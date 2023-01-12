#!/usr/bin/env julia

using Printf
using RobotOS

using StatsPlots, Distributions
using LinearAlgebra
using DelimitedFiles
using Plots
using Statistics
using Infiltrator
using Dates
using PlotThemes
using DataFrames

Plots.theme(:vibrant, fmt = :PDF)

@rosimport mrs_msgs.msg:RangeWithVar
rostypegen()
using .mrs_msgs.msg

data = Dict{String,DataFrame}()

START_TIME = datetime2unix(now())

function update_plots()
    histograms = []

    l = @layout [a{0.6h} ; b{0.2h}; c{0.2h}]

    begin_time = []
    for (key, value) in data
        push!(begin_time, min(value.timestamp ...))
    end

    START_TIME = min(begin_time...)

    distances = plot(
        minorgrid=true, 
        title="Range", 
        fontfamily="Computer Modern",
        titlefontsize=8,
        label="",
        xguide = "m",
        legend_position=:topleft,
        guidefontsize=6,
        color_palette=:lighttest
    )

    time_plot = plot(
        minorgrid=true, 
        title="Range in time", 
        fontfamily="Computer Modern",
        titlefontsize=8,
        label="",
        xguide = "time [s]",
        guidefontsize=6,
        legend_position=:topleft,
        color_palette=:lighttest
    )

    for (key, value) in data
        axis = stephist(last(value, 100).range, bins=11, normalize=:pdf, fill=true, fillalpha=0.5, label="", color=:red)
        if (size(value)[1] > 2)
            m = mean(last(value, 100).range)
            s = std(last(value, 100).range)

            d = Normal(m, s)
            plot!(axis, d, 
            linewidth=3,
            linestyle=:dash,
            color=:black,
            minorgrid=true, 
            title="$key", 
            legend_title="μ = $(@sprintf("%.2f", m)) m\nσ = $(@sprintf("%.3f", s)) m",
            legend_title_font_pointsize=6,
            legend_position=:topright,
            fontfamily="Computer Modern",
            titlefontsize=8,
            label="",
            guidefontsize=6,
            xguide = "m")
            plot!(distances, d, label=key, linewidth=2)
        end
        push!(histograms, axis)
        plot!(time_plot, value.timestamp .- START_TIME, value.range, label=key)
    end

    xlimits = xlims(time_plot)
    plot!(time_plot, xlims=(max(xlimits[1], xlimits[2]-60), xlimits[2]))

    xlimits = xlims(distances)
    plot!(distances, xlims=(0, max(10, xlimits[2])))

    pl = plot(plot(histograms...), distances, time_plot, layout = l, size=(1500,1000))
    
    display(pl)
end

function callback(msg::RangeWithVar)
    time_stamp = (convert(Float64, msg.stamp.secs) + msg.stamp.nsecs*1e-9)
    @printf "[%.3f] Range %.2f m from %s\n\r" time_stamp msg.range msg.uav_name

    if !haskey(data, msg.uav_name)
        data[msg.uav_name] = DataFrame(timestamp = Float64[], range = Float64[], var = Float64[])
    end

    push!(data[msg.uav_name], [time_stamp msg.range msg.variance])
end

function main()
    @printf "Julia visualizer\n\r"

    init_node("rosjl_example")
    sub = Subscriber{RangeWithVar}("/uav/uwb_range/range_out", callback)

    t = Timer((t) -> update_plots(), 1; interval=1/60) 

    spin()
end

try
    main()
catch err
    @printf "Fail\n"
    rethrow()
end