import pandas as pd
from styleframe import StyleFrame, utils

def is_cell_station(cell):
    if cell.style.bg_color == 'ffd966':
        return True
    else:
        return False

sf = StyleFrame.read_excel("cloudMap_v07.xlsx", read_style=True)
#print(sf)

print(is_cell_station(sf[4][0]))

print('------------------------------------')

df = pd.read_excel("cloudMap_v07.xlsx")

print(df)

#parse reverse-row order
#ignore first/last row
#ignore first col
#rev row = (len(df[col])-1)-row


for col in range(0,len(df)):
    for row in reversed(range(0,len(df[col])-1)):
    #for row in range(len(df[col])-2,0,-1):
        if(df[col][row] != 'x'):
            cell_type = "node"
            if(is_cell_station(sf[col][row])):
                cell_type = "station"
            print("    - !!python/tuple [" + str(col) +"," +  str((len(df)-2)-row)+"," + "\"" + str(df[col][row]) + "\"" +","+ "\"" + cell_type + "\"" +"]")