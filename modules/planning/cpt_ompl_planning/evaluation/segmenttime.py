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
    df.columns = ["scenario", "planner", "success", "duration", "length", "dist_surf", "smoothness", "segments", "rowid"]
    df["scenario"] = scenario
    df["duration"] = df["duration"] / 1000.0
    data.append(df)

all_data = pd.concat(data)


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
alpha_stripplot = 0.25
all_data = all_data[ all_data["planner"] == "RMP8"]
print((all_data["duration"]/all_data["segments"]).mean())

print(all_data["duration"].mean())