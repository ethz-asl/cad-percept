import numpy as np
from pandas import read_csv
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt
import seaborn as sns
from tabulate import tabulate
import copy

columns = [2, 3]
scenarios = ["curve", "hilo", "rhone"]
planners = ["RMP", "DGEO", "RRTConnect1000", "RRTStar1000", "RRTMeshProj1000", "RRTStar250", "RRTMeshProj250"]
planners_latex = ["\\textbf{Ours}", "DGEO", "\shortstack{RRT$^{*}$ \\\\ Con-$1$}",
                  "\shortstack{RRT$^{*}$ \\\\ Sam-$1$}",
                  "\shortstack{RRT$^{*}$ \\\\ Pro-$1$}", "\shortstack{RRT$^{*}$ \\\\ Sam-$\\sfrac{1}{4}$}",
                  "\shortstack{RRT$^{*}$ \\\\ Pro-$\\sfrac{1}{4}$}"]
data = []
for scenario in scenarios:
    df = read_csv(scenario + "_filtered.csv", delimiter="\t")
    df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness", "segments"]
    df["scenario"] = scenario
    df["duration"] = df["duration"] / 1000.0
    data.append(df)

all_data = pd.concat(data)
success_rates = all_data.groupby(["planner", "scenario"], as_index=False).agg(
    {"success": [lambda x: np.count_nonzero(x) / np.alen(x)],
     "duration": [np.average]})
success_rates.columns = ["planner", "scenario", "success_rate", "duration"]
print(success_rates)

default_rc = {'font.size': 8.0,
              'figure.figsize': (4, 4.0),
              'lines.linewidth': 2.6,
              'figure.facecolor': 'white',
              'axes.facecolor': 'white',
              'grid.color': 'grey',
              'grid.linewidth': 0.1,
              'axes.labelsize': 8.0,
              'axes.titlesize': 8.0,
              'legend.fontsize': 8.0,
              'xtick.labelsize': 8.0,
              'ytick.labelsize': 8.0,
              'text.usetex': True,
              'text.latex.preamble': [r'\usepackage{amsmath}',
                                      r'\usepackage{amssymb}',
                                      r'\usepackage{xfrac}'],
              'font.family': 'serif',
              'figure.constrained_layout.use': True}

sns.set_theme()
success_rates_rc = copy.deepcopy(default_rc)
success_rates_rc['figure.figsize'] = (4, 2.0)
success_rates_rc['figure.constrained_layout.h_pad'] = 0.05
success_rates_rc['figure.constrained_layout.w_pad'] = 0.2
sns.set(rc=success_rates_rc)
sns.set_palette("tab10")

success_rate_plot = sns.barplot(y='success_rate', x='planner',
                                data=success_rates,
                                order=planners,
                                alpha=1.0,
                                hue='scenario', linewidth=0)
success_rate_plot.legend(loc='upper center', borderaxespad=0., ncol=3, bbox_to_anchor=(0.5, 1.2))

success_rate_plot.set_xticklabels(planners_latex, rotation=90)
success_rate_plot.set(xlabel=None, ylabel='Success Rate $[\%]$')
# success_rate_plot.set(title="Planning Success Rate")
# plt.show()
figure = success_rate_plot.get_figure()
figure.savefig('/tmp/success_rate.pdf', dpi=300)
plt.clf()

# generate latex table
table = []
planner_header = copy.deepcopy(planners_latex)
planner_header.insert(0, "")
# generate header
table.append(planner_header)

# generate rows
for scenario in scenarios:
    row = [scenario]

    for planner in planners:
        filtered = success_rates[(success_rates["scenario"] == scenario) & (success_rates["planner"] == planner)][
            "success_rate"]
        assert len(filtered) == 1
        max_val = success_rates[(success_rates["scenario"] == scenario)]["success_rate"].max()
        curr_val = filtered.values[0]

        if curr_val == max_val:
            row.append("\\textbf{" + "{:.2f}".format(curr_val) + "}")
        else:
            row.append("{:.2f}".format(curr_val))
    table.append(row)
print(tabulate(table, tablefmt="latex_raw"))

# spl = sns.scatterplot(y='success_rate', x='duration',
#                      hue='planner',
#                      style='scenario',
#                      alpha=1.0,
#                      data=success_rates)
# spl.legend(loc='upper center', borderaxespad=0., ncol=2, bbox_to_anchor=(0.5, -.2))
#
# spl.get_figure().savefig('/tmp/rate_duration.png', dpi=300)
#
# plt.clf()

all_data = all_data[all_data["success"] == 1]
rmp_data = all_data[all_data["planner"] == "RMP"]

dist_scatter_rc = copy.deepcopy(default_rc)
dist_scatter_rc['figure.figsize'] = (4, 1.0)
# dist_scatter_rc['figure.constrained_layout.h_pad'] = 0.05
# dist_scatter_rc['figure.constrained_layout.w_pad'] = 0.2
sns.set(rc=dist_scatter_rc)
sns.set_palette("tab10")
bplot = sns.scatterplot(y='dist_surf', x='length',
                        hue='scenario',
                        marker='.',
                        alpha=0.8,
                        data=rmp_data,
                        linewidth=0.0)
bplot.set(xlabel='Path length [m]', ylabel='Avg. Dist to Surface [m]')
bplot.legend().remove()

bplot.get_figure().savefig('/tmp/dist_scatter.pdf', dpi=300)

# plt.clf()

plt.clf()
smoothness_rc = copy.deepcopy(default_rc)
smoothness_rc['figure.figsize'] = (4, 4.0)
smoothness_rc['figure.constrained_layout.h_pad'] = 0.05
smoothness_rc['figure.constrained_layout.w_pad'] = 0.2
sns.set(rc=smoothness_rc)
sns.set_palette("tab10")
bplot = sns.stripplot(y='smoothness', x='planner',
                      data=all_data,
                      order=planners,
                      jitter=True,
                      marker='o',
                      dodge=True,
                      alpha=0.3,
                      hue='scenario')
bplot.set(xlabel=None, ylabel='Smoothness')

bplot.set(ylim=(0.84, 1.02))
bplot.set_xticklabels(planners_latex, rotation=90)
bplot.legend().remove()

bplot.get_figure().savefig('/tmp/smoothness.pdf', dpi=300)

plt.clf()

duration_rc = copy.deepcopy(default_rc)
duration_rc['figure.figsize'] = (4, 3.0)
duration_rc['figure.constrained_layout.h_pad'] = 0.05
duration_rc['figure.constrained_layout.w_pad'] = 0.2
sns.set(rc=duration_rc)
sns.set_palette("tab10")
bplot = sns.stripplot(y='duration', x='planner',
                      data=all_data,
                      order=planners,
                      jitter=True,
                      marker='o',
                      dodge=True,
                      alpha=0.3,
                      hue='scenario')
bplot.set(xlabel=None, ylabel='Duration [ms]')
bplot.legend().remove()

bplot.set_xticklabels(planners_latex, rotation=90)
bplot.get_figure().savefig('/tmp/duration.pdf', dpi=300)
plt.clf()

dist_rc = copy.deepcopy(default_rc)
dist_rc['figure.figsize'] = (4, 3.0)
dist_rc['figure.constrained_layout.h_pad'] = 0.05
dist_rc['figure.constrained_layout.w_pad'] = 0.2
sns.set(rc=dist_rc)
sns.set_palette("tab10")
bplot = sns.stripplot(y='dist_surf', x='planner',
                      data=all_data,
                      order=planners,
                      jitter=True,
                      marker='o',
                      dodge=True,
                      alpha=0.5,
                      hue='scenario')
bplot.set_xticklabels(planners_latex, rotation=90)

bplot.set(xlabel=None, ylabel='Avg Surface Dist [m]')
bplot.legend().remove()
bplot.get_figure().savefig('/tmp/dist_surf.pdf', dpi=300)
plt.clf()

sns.scatterplot(y='smoothness', x='duration',
                hue='planner',
                marker='.',
                alpha=1.0,
                data=all_data).get_figure().savefig('/tmp/duration_smoothness.png', dpi=300)
plt.clf()

#
# plt.show()

# plt.show()

# print(all_data.groupby(["planner", "scenario"])["duration"].mean() / 1000.0)

# print(all_data.groupby(["planner", "scenario"])["smoothness"].mean())

# print(all_data.groupby(["planner", "scenario"])["dist_surf"].mean())

# print(all_data.groupby(["planner", "scenario"])["length"].mean())
