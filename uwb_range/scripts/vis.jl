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

@rosimport mrs_msgs.msg:RangeWithCovarianceArrayStamped
rostypegen()
using .mrs_msgs.msg

data = Dict{UInt64,DataFrame}()

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

function callback(msg::RangeWithCovarianceArrayStamped)
    time_stamp::Float64 = (convert(Float64, msg.header.stamp.secs) + msg.header.stamp.nsecs*1e-9)
    global START_TIME

    if ( isnan(START_TIME) || time_stamp < START_TIME)
        START_TIME = time_stamp
    end

    time_stamp = time_stamp - START_TIME

    for measurement in msg.ranges
        if (measurement.range.range < 0)
            return
        end

        @printf "[%.3f] Range %.2f m from 0x%X\n\r" time_stamp measurement.range.range measurement.id
    
        if !haskey(data, measurement.id)
            data[measurement.id] = DataFrame(timestamp = Float64[], range = Float64[], var = Float64[])
        end
    
        push!(data[measurement.id], [time_stamp measurement.range.range measurement.variance])
        filter!(row -> row.timestamp > time_stamp - 20, data[measurement.id])
    end
end

function main()
    @printf "Julia visualizer\n\r"

    init_node("rosjl_example")
    sub = Subscriber{RangeWithCovarianceArrayStamped}("/uav1/uwb_range/range", callback)

    t = Timer((t) -> update_plots(), 1; interval=1/25) 

    spin()
end

main()
