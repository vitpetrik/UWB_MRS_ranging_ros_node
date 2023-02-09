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
using DSP

Plots.theme(:vibrant, fmt = :PDF)

@rosimport mrs_msgs.msg:RangeWithVar
rostypegen()
using .mrs_msgs.msg

data = Dict{String,DataFrame}()

global START_TIME::Float64 = NaN

function update_plots()
    histograms = []

    l = @layout [a{0.6h} ; b{0.2h}; c{0.2h}]

    distances = plot(
        minorgrid=true, 
        title="Range", 
        fontfamily="Computer Modern",
        titlefontsize=12,
        label="",
        xguide = "m",
        legend_position=:topleft,
        guidefontsize=10,
        color_palette=:lighttest
    )

    time_plot = plot(
        minorgrid=true, 
        title="Range in time", 
        fontfamily="Computer Modern",
        titlefontsize=12,
        label="",
        xguide = "time [s]",
        guidefontsize=10,
        legend_position=:topleft,
        color_palette=:lighttest,
    )

    for (key, value) in data
        axis = stephist(last(value, 500).range, bins=11, normalize=:pdf, fill=true, fillalpha=0.5, label="", color=:red)
        if (size(value)[1] > 2)
            m = mean(last(value, 500).range)
            s = std(last(value, 500).range)

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
            titlefontsize=12,
            label="",
            guidefontsize=10,
            xguide = "m")
            plot!(distances, d, label=key, linewidth=2)
        end
        push!(histograms, axis)
        plot!(time_plot, value.timestamp, value.range, label=key)
    end

    xlimits = xlims(distances)
    plot!(distances, xlims=(0, max(10, xlimits[2])))

    pl = plot(plot(histograms...), distances, time_plot, layout = l, size=(1500,1000))
    
    display(pl)
end

function callback(msg::RangeWithVar)
    time_stamp::Float64 = (convert(Float64, msg.stamp.secs) + msg.stamp.nsecs*1e-9)
    global START_TIME

    if ( isnan(START_TIME) || time_stamp < START_TIME)
        START_TIME = time_stamp
    end

    if (msg.range < 0)
        return
    end

    time_stamp = time_stamp - START_TIME

    @printf "[%.3f] Range %.2f m from %s\n\r" time_stamp msg.range msg.uav_name

    if !haskey(data, msg.uav_name)
        data[msg.uav_name] = DataFrame(timestamp = Float64[], range = Float64[], var = Float64[])
    end

    push!(data[msg.uav_name], [time_stamp msg.range msg.variance])
    filter!(row -> row.timestamp > time_stamp - 20, data[msg.uav_name])
end

function main()
    @printf "Julia visualizer\n\r"

    init_node("rosjl_example")
    sub = Subscriber{RangeWithVar}("/uav/uwb_range/range_out", callback)

    t = Timer((t) -> update_plots(), 1; interval=1/25) 

    spin()
end

main()