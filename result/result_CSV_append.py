import pandas as pd

import os

# Set the directory to search for CSV files

map_name = '_map_bugtrap'
map_name = '_map_deadend'
experiment_title = 'experiment2'
directory = os.path.join(experiment_title, map_name)

# List all files in the directory
files = os.listdir(directory)

# Filter the list of files to only include CSV files
csv_files = [f for f in files if f.find('_OUR_RRTX.csv') != -1]

# Print the list of CSV files
first = True
for f in csv_files:
    filename = os.path.join(directory, f)
    df1 = pd.read_csv(filename)
    if first:
        first = False
        df = df1
    else:
        # Concatenate the two DataFrames
        df = pd.concat([df, df1], ignore_index=True)

# Write the concatenated DataFrame to a new CSV file
df.to_csv(f'{experiment_title}{map_name}.csv', index=False)