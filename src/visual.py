import pandas as pd

# Define the number of experiments and algorithms
num_experiments = 20
algorithms = ['RRT', 'RRTC', 'RRT*']

# Initialize an empty DataFrame with appropriate columns
# The DataFrame will have one column for each algorithm, with each row representing an experiment
df = pd.DataFrame(columns=algorithms, index=[f"Experiment_{i+1}" for i in range(num_experiments)])

# df['RRT'] = []  # Example times
df['RRTC'] = [817.03,1000,1608,2051,1327]  # Example times
# df['RRT*'] = []



# Display the empty DataFrame structure
print("DataFrame structure ready for data entry:")
print(df)

# Save to CSV (optional)
# df.to_csv('experiment_times.csv', index=True)
