import yfinance as yf
import pandas as pd

ticker_symbol = "^GSPC"  # Yahoo Finance symbol for S&P500

data = yf.download(ticker_symbol, start="2023-01-01", end="2024-01-01")

# Reset index to ensure first row is in correct format
df = data.reset_index()

df.columns = ["Date", "Adj Close", "Close", "High", "Low", "Open", "Volume"]

df.to_csv("clean_sp500_data.csv", index=False)