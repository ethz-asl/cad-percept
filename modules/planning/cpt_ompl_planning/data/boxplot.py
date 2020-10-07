import numpy as np
from pandas import read_csv
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

columns = [2, 3]
scenarios = ["curve", "hilo", "rhone"]
planners = ["RMP", "DGEO", "RRTConnect1000", "RRTStar1000", "RRTMeshProj1000", "RRTStar250", "RRTMeshProj250"]

data = []
for scenario in scenarios:
    df = read_csv(scenario + "_filtered.csv", delimiter="\t")
    df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness", "segments"]
    df["scenario"] = scenario
    data.append(df)

all_data = pd.concat(data)
success_rates = all_data.groupby(["planner", "scenario"], as_index=False).agg(
    {"success": [lambda x: np.count_nonzero(x) / np.alen(x)]})
success_rates.columns = ["planner", "scenario", "success_rate"]
print(success_rates)
sns.set_style("white")

success_rate_plot = sns.barplot(y='success_rate', x='planner',
                                data=success_rates,
                                order=planners,
                                alpha=0.5,
                                hue='scenario')
success_rate_plot.set_xticklabels(success_rate_plot.get_xticklabels(), rotation=30)
success_rate_plot.set(xlabel='Planner', ylabel='Planning Success Rate [%]')

plt.show()

all_data = all_data[all_data["success"] == 1]

bplot = sns.stripplot(y='smoothness', x='planner',
                      data=all_data,
                      order=planners,
                      jitter=True,
                      marker='o',
                      dodge=True,
                      alpha=0.5,
                      hue='scenario')
bplot.set(xlabel='Planner', ylabel='Smoothness')
bplot.set(ylim=(0.9, 1.0))
bplot.set_xticklabels(bplot.get_xticklabels(), rotation=30)
plt.show()

bplot = sns.stripplot(y='duration', x='planner',
                      data=all_data,
                      order=planners,
                      jitter=True,
                      marker='o',
                      dodge=True,
                      alpha=0.5,
                      hue='scenario')
bplot.set(xlabel='Planner', ylabel='Duration [us]')
# bplot.set(ylim=(0.9, 1.0))
bplot.set_xticklabels(bplot.get_xticklabels(), rotation=30)
plt.show()

bplot = sns.stripplot(y='dist_surf', x='planner',
                      data=all_data,
                      order=planners,
                      jitter=True,
                      marker='o',
                      dodge=True,
                      alpha=0.5,
                      hue='scenario')
bplot.set(xlabel='Planner', ylabel='Avg Surface Dist [m]')
# bplot.set(ylim=(0.9, 1.0))
bplot.set_xticklabels(bplot.get_xticklabels(), rotation=30)
plt.show()

sns.scatterplot(y='smoothness', x='duration',
                hue='planner',
                data=all_data)
plt.show()




#
# plt.show()

# plt.show()

# print(all_data.groupby(["planner", "scenario"])["duration"].mean() / 1000.0)

# print(all_data.groupby(["planner", "scenario"])["smoothness"].mean())

# print(all_data.groupby(["planner", "scenario"])["dist_surf"].mean())

# print(all_data.groupby(["planner", "scenario"])["length"].mean())
