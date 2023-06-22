import pandas as pd

filename = 'data.csv'
df = pd.read_csv(filename)

new_row = {'A': 100, 'B': 200, 'C': 300}  # Example dictionary for a new row

df = df.append(new_row, ignore_index=True)