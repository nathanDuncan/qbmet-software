import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file. Make sure "data.csv" is in the same folder as this script.
for i in range(13, 14):
    df = pd.read_csv(f'Dataset_0{i}.csv')

    # drop the first row
    df = df.drop([0])

    # remove all nan
    df = df.dropna()

    print(df.head())
    # The time column will be used as the x-axis.
    # Assume the CSV header includes "Time_ms" as the first column
    time = df['Time_ms']

    # Get all variable names except "Time_ms"
    variables = df.columns.drop('Time_ms')

    # Loop over each variable and plot it in its own figure
    for var in variables:
        plt.figure()              # Create a new figure window
        plt.plot(time, df[var], 'o', label=var)
        plt.title(var)            # Set the title to the variable name
        plt.xlabel('Time (ms)')
        plt.ylabel(var)
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.show()  # This call blocks until you close the figure window

    print("All plots displayed.")