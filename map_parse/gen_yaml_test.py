import pandas as pd

df = pd.read_excel("cloudMap_v07.xlsx")

print(df)

#parse reverse-row order
#ignore first/last row
#ignore first col
#rev row = (len(df[col])-1)-row

for col in range(0,len(df)):
    for row in range(len(df[col])-2,0,-1):
        if(df[row][col] != 'x'):
            print("    - !!python/tuple [" + str(col) +"," +  str((len(df)-2)-row)+","+ str(df[row][col])+","+ "node]")