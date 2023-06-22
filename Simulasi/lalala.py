import pandas as pd
filename = 'data.csv'
df = pd.read_csv(filename)
max_value = df['Column_Name'].max()
