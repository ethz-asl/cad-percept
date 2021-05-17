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
planners = ["RMP8", "DGEO", "RRTConnect1000", "RRTStar1000", "RRTMeshProj1000", "RRTStar250", "RRTMeshProj250"]
planners_latex = ["\\textbf{Ours}", "DGEO", "\shortstack{RRT$^{*}$ \\\\ Con-$1$}",
                  "\shortstack{RRT$^{*}$ \\\\ Sam-$1$}",
                  "\shortstack{RRT$^{*}$ \\\\ Pro-$1$}", "\shortstack{RRT$^{*}$ \\\\ Sam-$\\sfrac{1}{4}$}",
                  "\shortstack{RRT$^{*}$ \\\\ Pro-$\\sfrac{1}{4}$}"]
data = []
for scenario in scenarios:
    df = read_csv(scenario + "_data.log", delimiter="\t")
    df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness", "segments","rowid"]
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


#all_data = all_data[all_data["success"] == 1]
dgeo = all_data[all_data["planner"] == "DGEO"]

# calculate relative length for all rows!
rel_length = []
for index, row in all_data.iterrows():
    if row["success"] == 0:
        rel_length.append(0)
    else:
        length = row["length"]
        dgeo_length_rows = all_data[(all_data["planner"] == "DGEO") & (all_data["scenario"] == row["scenario"]) &
                               (all_data["rowid"] == row["rowid"])]["length"]
        assert(len(dgeo_length_rows) == 1)
        dgeo_length = dgeo_length_rows.values[0]
        length_ratio = length/dgeo_length
        rel_length.append(length_ratio)

all_data = all_data.assign(len_ratio = rel_length)
print(all_data)


