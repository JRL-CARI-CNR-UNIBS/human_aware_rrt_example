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
from scipy import stats
import seaborn as sns


# Params set by user
load_from_parameter_server=False    # load current rosparam (for online analysis)
only_if_different=True              # skip results that are equal to the baseline (i.e. hamp was not activated)
use_median=True                     # group repetitions and compute median before normalization
itp_delay=0.35                      # delay of the time parametrization (used to adjust the scaling values)

dof=3
test_name='hamp_result_20211007_100queries_02m.yaml'

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

planner_map = {'hamp_timebased': 'HAMP', 'timebased': 'MIN-TIME', 'mixed_strategy_a' : 'MIN-PATH'}
planner_labels=[planner_map[x] for x in planner_ids]
print(planner_labels)

start_query=24
if load_from_parameter_server:
    start_query=rospy.get_param("/starting_trial")

lengths_normalized=[]
times_exec_normalized=[]
times_nominal_normalized=[]
slowdowns_normalized=[]
outcomes=[]

lengths_raw=[]
times_exec_raw=[]
times_nominal_raw=[]
slowdowns_raw=[]

lengths_fails=[]
outcomes_fails=[]

baseline_failed = False

for i_planner,planner in (enumerate(planner_ids)):
    lengths_normalized.append([])
    times_exec_normalized.append([])
    times_nominal_normalized.append([])
    slowdowns_normalized.append([])
    outcomes.append([])
    #
    lengths_raw.append([])
    times_exec_raw.append([])
    times_nominal_raw.append([])
    slowdowns_raw.append([])
    #
    lengths_fails.append([])
    outcomes_fails.append([])

for iquery in range(start_query,queries_number):
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
            if (result["outcome"]==1 and result["trajectory_length"]>0): # length is < 0 if query was skipped
                lengths.append(result["trajectory_length"])
                times_exec.append(result["trajectory_time"])
                times_nominal.append(result["trajectory_nominal_time"])
                planning_time.append(result["planning_time"])
                average_slowdown.append(result["average_slowdown"]-(itp_delay/result["trajectory_nominal_time"]))
            elif result["trajectory_length"]>0.0:
                lengths.append(result["trajectory_length"]) # append planned length even if failed
                #print("query: " + str(iquery) + "rep: " + str(i_rep) + " planner: " + planner + "length: " + str(result["trajectory_length"]))

        outcomes[i_planner].extend(outcome)

        if i_planner==0: # the first planner is the baseline
            if sum(outcome)==0: # baseline failed
                baseline_failed=True
                length_median_baseline=statistics.median(lengths)
            else:
                baseline_failed = False
                length_median_baseline=statistics.median(lengths)
                time_exec_median_baseline = statistics.median(times_exec)
                time_nominal_median_baseline = statistics.median(times_nominal)
                slowdown_median_baseline = statistics.median(average_slowdown)

        if not baseline_failed and sum(outcome)>0:
            if ((not only_if_different) or abs(statistics.median(lengths)-length_median_baseline)/length_median_baseline>=0.02 or i_planner<1):
                if use_median==True:
                    lengths_normalized[i_planner].append( statistics.median(lengths) / length_median_baseline  )
                    times_exec_normalized[i_planner].append( statistics.median(times_exec) / time_exec_median_baseline )
                    times_nominal_normalized[i_planner].append( statistics.median(times_nominal) / time_nominal_median_baseline )
                    slowdowns_normalized[i_planner].append( statistics.median(average_slowdown) / slowdown_median_baseline )
                else:
                    lengths_normalized[i_planner].extend( [x / length_median_baseline for x in lengths] )
                    times_exec_normalized[i_planner].extend( [x / time_exec_median_baseline for x in times_exec] )
                    times_nominal_normalized[i_planner].extend( [x / time_nominal_median_baseline for x in times_nominal] )
                    slowdowns_normalized[i_planner].extend([x / slowdown_median_baseline for x in average_slowdown])
        elif sum(outcome)>0:
            if ((not only_if_different) or abs(statistics.median(lengths)-length_median_baseline)/length_median_baseline>=0.02 or i_planner<2):
                if use_median==True:
                    lengths_normalized[i_planner].append( statistics.median(lengths) / length_median_baseline  )
                else:
                    lengths_normalized[i_planner].extend( [x / length_median_baseline for x in lengths] )

        # append pairs for wilcoxon p-test
        if not baseline_failed and sum(outcome)>0:
            lengths_raw[i_planner].append(statistics.mean(lengths))
            times_exec_raw[i_planner].append(statistics.mean(times_exec))
            times_nominal_raw[i_planner].append(statistics.mean(times_nominal))
            slowdowns_raw[i_planner].append(statistics.mean(average_slowdown))

        # append failures
        if (baseline_failed or sum(outcome)==0) and len(lengths)>0:
            lengths_fails[i_planner].append(statistics.median(lengths)/length_median_baseline)
            if (max(outcome)==0):
                outcomes_fails[i_planner].append("failed")
            else:
                outcomes_fails[i_planner].append("success")


# Wilcoxon paired signed test_name
for i_planner,planner in enumerate(planner_ids):
    if(i_planner>0):
        print(len(lengths_raw[0]))
        print(len(lengths_raw[i_planner]))
        print("Wilcoxon results: " + planner + " vs "+planner_ids[0])
        #
        print("length",stats.wilcoxon( lengths_raw[0] , lengths_raw[i_planner] ,"pratt"))
        print("exec_time",stats.wilcoxon(times_exec_raw[0], times_exec_raw[i_planner],"pratt"))
        print("nominal_time",stats.wilcoxon(times_nominal_raw[0], times_nominal_raw[i_planner],"pratt"))
        print("slowdown",stats.wilcoxon(slowdowns_raw[0], slowdowns_raw[i_planner],"pratt"))

# plot length vs exec_time
fig1, ax1 = plt.subplots(1,1)
ax1.plot(lengths_raw[0], times_nominal_raw[0],'+', color=(0,0.6,0,0.4),label=planner_labels[0]+" (nom. time)")
x=lengths_raw[0]
y=times_nominal_raw[0]
ax1.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)),'--',color=(0,0.6,0,0.9),linewidth=2)

ax1.plot(lengths_raw[0], times_exec_raw[0],'o', color=(0,0,0.5,0.3),label=planner_labels[0]+" (exec. time)")
x=lengths_raw[0]
y=times_exec_raw[0]
ax1.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)),'--',color=(0,0,0.5,0.9),linewidth=2)

for i_planner,planner in enumerate(planner_ids):
    if(i_planner>0):
        ax1.plot(lengths_raw[i_planner], times_exec_raw[i_planner],'^',color=(1,0.5,0,0.4),label=planner_labels[i_planner]+" (exec. time)")
        x=lengths_raw[i_planner]
        y=times_exec_raw[i_planner]
        ax1.plot(np.unique(x), np.poly1d(np.polyfit(x, y, 1))(np.unique(x)),'--',color=(1,0.5,0,0.9),linewidth=2)
ax1.xaxis.grid(zorder=0)
ax1.yaxis.grid(zorder=0)
ax1.set_axisbelow(True)

plt.xlim([0.5,3.5])
plt.ylim([1,7])
plt.xlabel('path length [rad]')
plt.ylabel('execution time [s]')
plt.legend()

plt.show(block = False)
plt.pause(0.001)

plt.savefig('./length_vs_time_plot.pdf', format='pdf')



fig2, ax2 = plt.subplots(1,1)
c_map = {'failed': 'r', 'success': 'g'}
m_map = {'failed': 'x', 'success': '*'}

for i_planner,planner in enumerate(planner_ids):
    X=lengths_fails[-1]
    Y=lengths_fails[i_planner]
    Z=outcomes_fails[i_planner]
    y=list(zip(*(sorted(zip(X,Y)))))[1]
    hue=list(zip(*(sorted(zip(X,Z)))))[1]
    x=range(1,len(y)+1)
    if i_planner==0:
        mark='_'
    else:
        mark='s'
    ax2.scatter(x, y, c=[c_map[x] for x in hue], marker=mark,label=planner_labels[i_planner])
ax2.yaxis.grid(zorder=0)
ax2.xaxis.grid(zorder=0)
ax2.set_axisbelow(True)

plt.xlabel('n. test (only failures considered)')
plt.ylabel('length hamp / length min-path')
plt.legend()


plt.show(block = False)
plt.pause(0.001)

plt.savefig('./failures_visualization.pdf', format='pdf')


# compact data for boxplots
length_array=[]
time_exec_array=[]
time_nominal_array=[]
slowdown_array=[]
outcome_array=[]

gain_lengths=[1,1,1,1]

for i_planner,planner in enumerate(planner_ids):
    length_array.append( [ x*gain_lengths[i_planner] for x in lengths_normalized[i_planner] ] )
    time_exec_array.append(times_exec_normalized[i_planner])
    time_nominal_array.append(times_nominal_normalized[i_planner])
    slowdown_array.append(slowdowns_normalized[i_planner])
    outcome_array.append(outcomes[i_planner])


datas = [length_array, time_exec_array, time_nominal_array, slowdown_array, outcome_array]
xlabels = planner_labels
titles=["path length", "execution time", "nominal time", "safety speed delay", "success rate"]

select_indices=[0,1,3,4]

medianprops = dict(color="navy", linewidth=1.5)

colors = ['aquamarine', 'paleturquoise', 'turquoise','blue']

fig, axes = plt.subplots(nrows=1, ncols=len(select_indices), figsize=(25, 5))
for ax,id in zip(axes,select_indices):
    title=titles[id]
    data=datas[id]

    ax.set_title(title)

    ax.yaxis.grid(zorder=0)
    ax.set_axisbelow(True)

    if title=="success rate":
        barplot = ax.bar(range(len(xlabels)), [(float(sum(x))/float(len(x))) for x in data],
                         tick_label = xlabels,
                         color = colors,
                         edgecolor="black",
                         fill=True,
                         zorder=3)
        ax.set(ylim=(0.0, 1.02))
        ax.set_ylabel("[n. success tests / n. total tests]")


    else:
        bplot = ax.boxplot(data[1:],
                           notch=False,  # notch shape
                           vert=True,  # vertical box alignment
                           patch_artist=True,  # fill with color
                           labels=xlabels[1:],  # will be used to label x-ticks
                           medianprops=medianprops,
                           showmeans=True,
                           widths=0.45,
                           showfliers=True)
        ax.plot([0,2],[1,1],'--',color="navy",linewidth=2)
        ax.set(ylim=(0.0, 2.5))
        ax.set_ylabel('[normalized w.r.t. min-path]')
        #ax.set(xlim=(-1, 1))

        for patch, color in zip(bplot['boxes'], colors[1:]):
            patch.set_facecolor(color)






    print(title+":")
    if title == "success rate":
        for i_pl, planner in enumerate(data):
            print("\t" + xlabels[i_pl] + ": " + "{:.3f}".format(float(sum(planner)) / float(len(planner))) + "(n=" + str(len(planner)) + ")")
    else:
        for i_pl,planner in enumerate(data):
            print( "\t" + xlabels[i_pl]+": " + "{:.3f}".format(statistics.mean(planner)) + " \pm "+ "{:.4f}".format(statistics.stdev(planner)) + "(median = " + "{:.3f}".format(statistics.median(planner)) + "n=" + str(len(planner)) + ")")





#plt.ylim([0.999,1.001])

plt.savefig('./hamp_results.pdf', format='pdf')

print("Plotting results for "+str(queries_number) + " queries of " + str(repetitions) + " repetitions.")


plt.show()
