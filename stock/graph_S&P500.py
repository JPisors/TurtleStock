import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("stock/clean_sp500_data.csv")

df['Date'] = pd.to_datetime(df['Date'])

# Plotting the day's close per day
plt.figure(figsize=(10, 6))
plt.plot(df['Date'], df['Close'], label='S&P 500 Close')

plt.title("S&P 500 Close Price Over Time")
plt.xlabel("Date")
plt.ylabel("Close Price (USD)")
plt.xticks(rotation=45)
plt.legend()
plt.show() # Note: close graph and python will stop running