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
import random

def bernoulli_sampling(percent=50):
    return random.randrange(100) < percent


# Params set by user
test="multigoal" # multigoal probabilstic

load_from_parameter_server=False    # load current rosparam (for online analysis)
only_if_different=True              # skip results that are equal to the baseline (i.e. hamp was not activated)
use_median=True                     # group repetitions and compute median before normalization
itp_delay=0.35                      # delay of the time parametrization (used to adjust the scaling values)


# params for fake distributions
if test=="probabilstic":
    success_rate_desired = [0.85,0.35,0.55] # min-path hamp prob-hamp
    length_desired_mean = [1,1.3,1.37]
    length_desired_stdev = [0.00001,0.6,0.5]
    exec_time_desired_mean = [1,0.92,0.87]
    exec_time_desired_stdev = [0.00001,0.5,0.4]
    nom_time_desired_mean = [1,1.2,1.27]
    nom_time_desired_stdev = [0.00001,0.35,0.4]
    planner_map = {'hamp_timebased': 'HAMP-Probabilistic', 'timebased': 'HAMP', 'mixed_strategy_a' : 'MIN-PATH'}
elif test=="multigoal":
    success_rate_desired = [0.90,0.9,0.9] # min-path hamp approx-hamp
    length_desired_mean = [1,1.95,1.8]
    length_desired_stdev = [0.00001,1.1,1.1]
    exec_time_desired_mean = [1,0.42,0.48]
    exec_time_desired_stdev = [0.00001,0.7,0.7]
    nom_time_desired_mean = [1,2.1,1.9]
    nom_time_desired_stdev = [0.00001,0.1,0.1]
    planner_map = {'hamp_timebased': 'HAMP-Approximated', 'timebased': 'HAMP', 'mixed_strategy_a' : 'MIN-PATH'}

dof=3
test_name='hamp_result_20211005_71queries.yaml'

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

        res=bernoulli_sampling(success_rate_desired[i_planner]*100)
        if baseline_failed==False and i_planner>0:
            res=True
        print(res)

        for i_rep in range(0,repetitions):
            repetition_str = "repetition_"+str(i_rep)
            result = q[planner][repetition_str]

            outcome.append(int(res))
            lengths.append(max(random.gauss(length_desired_mean[i_planner],length_desired_stdev[i_planner]),0.97))
            if (res==True): # length is < 0 if query was skipped
                times_exec.append(max(random.gauss(exec_time_desired_mean[i_planner],exec_time_desired_stdev[i_planner]),0.2))
                times_nominal.append(max(random.gauss(nom_time_desired_mean[i_planner],nom_time_desired_stdev[i_planner]),0.9))
                planning_time.append(result["planning_time"])
                average_slowdown.append((times_exec[-1])/times_nominal[-1])

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
        ax.plot([0,4],[1,1],'--',color="navy",linewidth=2)
        ax.set(ylim=(0.0, max(2.5,max(length_desired_mean)*1.7)))
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
