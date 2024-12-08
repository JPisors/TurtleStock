import pandas as pd

# To get the data ready for the turtlesim node

df = pd.read_csv("stock/refined_sp500_data.csv")

# Normalize the index values (250 total) to the x coordinates (0-11) of turtlesim
df['Scaled_X'] = df.index / (len(df) - 1) * 11

# Find the range of the price and then normalize it
close_min = df['Close'].min()
close_max = df['Close'].max()
df['Scaled_Y'] = (df['Close'] - close_min) / (close_max - close_min) * 11

df.to_csv("scaled_sp500_data.csv", index=False)