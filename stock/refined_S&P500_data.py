import pandas as pd

df = pd.read_csv("clean_sp500_data.csv")

# Keeping only Date and Close columns
df = df[['Date', 'Close']]

df['Date'] = pd.to_datetime(df['Date'])

# Adding an index column (date's skip weekends, this will be easier to convert to ROS movement)
df = df.reset_index(drop=True)
df['Index'] = df.index 

# Removes decimals/rounds
df['Close'] = df['Close'].astype(int)

df.to_csv("refined_sp500_data.csv", index=False)