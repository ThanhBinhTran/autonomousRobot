import pandas as pd

import os

# Set the directory to search for CSV files
#map_name = '_map_bugtrap'
map_name = '_map_deadend'
experiment_title = 'experiment1'
directory = os.path.join('result', experiment_title, map_name)

# List all files in the directory
files = os.listdir(directory)

# Filter the list of files to only include CSV files
csv_files = [f for f in files if f.find('improve*.csv') != -1]
print (csv_files)
# Print the list of CSV files
fname = f's(0_0)_g(20_100)_range20_ASP_AStar_RRTStar_path_cost_improveFalse.csv'
fname1 = f's(0_0)_g(20_100)_range20_ASP_AStar_RRTStar_path_cost_improveTrue.csv'

fullname = os.path.join(directory, fname)
if os.path.exists(fullname):
    df0 = pd.read_csv(fullname)

fullname1 = os.path.join(directory, fname1)
if os.path.exists(fullname1):
    df1 = pd.read_csv(fullname1)

first = True
for f in csv_files:
    filename = os.path.join(directory, f)
    
    if first:
        first = False
        df = df1
    else:
        # Concatenate the two DataFrames
        df = pd.concat([df, df1], ignore_index=True)

# Write the concatenated DataFrame to a new CSV file
df.to_csv(f'{experiment_title}{map_name}_g(20_100)_improve.csv', index=False)