#!/usr/bin/env julia

using Printf
using RobotOS

using StatsPlots, Distributions
using LinearAlgebra
using DelimitedFiles
using Plots, Measures
using Statistics
using Infiltrator
using Dates
using PlotThemes
using DataFrames
using DSP

@rosimport mrs_msgs.msg:RangeWithCovarianceArrayStamped
rostypegen()
using .mrs_msgs.msg

gr()


data = Dict{UInt64,DataFrame}()
global index = 0

global START_TIME::Float64 = NaN

function update_plots()
    default_plots = (
        fontfamily="computer modern",
        minorgrid=true,
        legend=true,
        color_palette=palette(["#43F6C8", "#2989D8", "#9A9EFF", "#1FCAFF"]),
        widen=true,
        background_color=:white,
    )
    global index
    histograms = []

    l = @layout [a{0.6h} ; b{0.2h}; c{0.2h}]

    distances = plot(
        title="Vzdálenosti", 
        fontfamily="Computer Modern";
        default_plots...,
        label="",
        xguide = "m",
        legend_position=:topleft,
        )

    time_plot = plot(
        title="Vzdálenost v čase", 
        fontfamily="Computer Modern";
        default_plots...,
        label="",
        xguide = "čas [s]",
        legend_position=:topleft,
        )

    for (key, value) in data
        axis = stephist(last(value, 500).range, bins=11, normalize=:pdf, fill=true, fillalpha=1.0, label="", color="#43F6C8"; default_plots...)
        if (size(value)[1] > 2)
            m = mean(last(value, 500).range)
            s = std(last(value, 500).range)

            d = Normal(m, s)
            plot!(axis, d;
            default_plots..., 
            linewidth=3,
            linestyle=:dash,
            color=:black,
            title="$key", 
            legend_title="μ = $(@sprintf("%.2f", m)) m\nσ = $(@sprintf("%.3f", s)) m",
            legendfontsize=10,
            legend_position=:topright,
            fontfamily="Computer Modern",
            label="",
            xguide = "vzdálenost [m]")
            plot!(distances, d, label=key, linewidth=2)
        end
        push!(histograms, axis)
        plot!(time_plot, value.timestamp.-value.timestamp[1], value.range, label=key)
    end

    xlimits = xlims(distances)
    plot!(distances, xlims=(0, max(5, xlimits[2])))

    pl = plot(plot(histograms...), distances, time_plot, layout = l; default_plots..., size=(1024,768), margin = 5mm)
    
    savefig(pl, "plot_$(index).pdf")
    index += 1
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

        if measurement.range.range > 5
            return
        end
    
        push!(data[measurement.id], [time_stamp measurement.range.range measurement.variance])
        filter!(row -> row.timestamp > time_stamp - 60, data[measurement.id])
    end
end

function main()
    @printf "Julia visualizer\n\r"

    init_node("rosjl_example")
    sub = Subscriber{RangeWithCovarianceArrayStamped}("/uav1/uwb_range/range", callback)

    t = Timer((t) -> update_plots(), 1; interval=10) 

    spin()
end

main()
