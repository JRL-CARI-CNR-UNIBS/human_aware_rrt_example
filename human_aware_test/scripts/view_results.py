#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""

author: hypatia
"""

import yaml
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt
import statistics
import seaborn
import pandas as pd

# Params set by user
load_from_parameter_server=True     # load current rosparam (for online analysis)
only_if_different=False             # skip results that are equal to the baseline (i.e. hamp was not activated)
itp_delay=0.35                      # delay of the time parametrization (used to adjust the scaling values)

dof=3
test_name='hamp_result.yaml'

if load_from_parameter_server:
   param=rospy.get_param("/hamp")
else:
   with open(test_name) as f:
       param = yaml.safe_load(f)

queries_number=param["queries_executed"]
repetitions=param["repetitions"]
#query_prefix=param["prefix"]

tested_planners=[]
planner_str=[]
planner_ids=param["planners"]

lengths_normalized=[]
times_exec_normalized=[]
times_nominal_normalized=[]
slowdowns_normalized=[]
outcomes=[]

baseline_failed = False

for i_planner,planner in (enumerate(planner_ids)):
    lengths_normalized.append([])
    times_exec_normalized.append([])
    times_nominal_normalized.append([])
    slowdowns_normalized.append([])
    outcomes.append([])

for iquery in range(0,queries_number):
    query_str="query_"+str(iquery)
    q=param[query_str]

    for i_planner,planner in enumerate(planner_ids):

        lengths=[]
        times_exec=[]
        times_nominal=[]
        planning_time=[]
        outcome=[]
        average_slowdown=[]

        for i_rep in range(0,repetitions):
            repetition_str = "repetition_"+str(i_rep)
            result = q[planner][repetition_str]

            outcome.append(int(result["outcome"]))
            if result["outcome"]==1:
                lengths.append(result["trajectory_length"])
                times_exec.append(result["trajectory_time"])
                times_nominal.append(result["trajectory_nominal_time"])
                planning_time.append(result["planning_time"])
                average_slowdown.append(result["average_slowdown"]-(itp_delay/result["trajectory_nominal_time"]))

        outcomes[i_planner].extend(outcome)

        if i_planner==0: # the first planner is the baseline
            if len(lengths)==0: # baseline failed
                baseline_failed=True
            else:
                baseline_failed = False
                length_median_baseline=statistics.mean(lengths)
                time_exec_median_baseline = statistics.mean(times_exec)
                time_nominal_median_baseline = statistics.mean(times_nominal)
                slowdown_median_baseline = statistics.mean(average_slowdown)

        if not baseline_failed:
            if abs(statistics.mean(lengths)-length_median_baseline)>=1e-2 or (not only_if_different): # or i_planner!=3:
                lengths_normalized[i_planner].extend( [x / length_median_baseline for x in lengths] )
                times_exec_normalized[i_planner].extend( [x / time_exec_median_baseline for x in times_exec] )
                times_nominal_normalized[i_planner].extend( [x / time_nominal_median_baseline for x in times_nominal] )
                slowdowns_normalized[i_planner].extend([x / slowdown_median_baseline for x in average_slowdown])

# compact data for boxplots
length_array=[]
time_exec_array=[]
time_nominal_array=[]
slowdown_array=[]
outcome_array=[]

for i_planner,planner in enumerate(planner_ids):
    length_array.append(lengths_normalized[i_planner])
    time_exec_array.append(times_exec_normalized[i_planner])
    time_nominal_array.append(times_nominal_normalized[i_planner])
    slowdown_array.append(slowdowns_normalized[i_planner])
    outcome_array.append(outcomes[i_planner])


datas = [length_array, time_exec_array, time_nominal_array, slowdown_array, outcome_array]
xlabels = planner_ids
titles=["length", "time_exec", "time_nominal", "slowdown", "success rate"]

medianprops = dict(color="navy", linewidth=1.5)

fig, axes = plt.subplots(nrows=1, ncols=len(datas), figsize=(15, 3.5))

for ax, title, data in zip(axes, titles, datas):
    bplot = ax.boxplot(data,
                       notch=False,  # notch shape
                       vert=True,  # vertical box alignment
                       patch_artist=True,  # fill with color
                       labels=xlabels,  # will be used to label x-ticks
                       medianprops=medianprops,
                       showmeans=True,
                       showfliers=False)
    ax.set_title(title)
    ax.yaxis.grid(True)
    colors = ['aquamarine', 'paleturquoise', 'turquoise']
    for patch, color in zip(bplot['boxes'], colors):
        patch.set_facecolor(color)

    print(title+":")
    for i_pl,planner in enumerate(data):
        print( "\t" + xlabels[i_pl]+": " + "{:.3f}".format(statistics.mean(planner)) + " \pm "+ "{:.4f}".format(statistics.stdev(planner)) )

#plt.ylim([0.999,1.001])

plt.savefig('./hamp_results.pdf', format='pdf')

print("Plotting results for "+str(queries_number) + " queries of " + str(repetitions) + " repetitions.")
plt.show()