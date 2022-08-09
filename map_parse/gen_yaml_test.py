import pandas as pd
from styleframe import StyleFrame, utils

def is_cell_station(cell):
    if cell.style.bg_color == 'ffd966' or cell.style.bg_color == 'FFFFD966': #station
        return True
    
    elif cell.style.bg_color == 'bcd6ee' or cell.style.bg_color == 'FF9DC3E6': #charge station
        return True

    else:
        return False

excel_file = "cloudMap_v10.xlsx"

sf = StyleFrame.read_excel(excel_file, read_style=True)
#print(sf)

print(is_cell_station(sf[0][0]))

print('------------------------------------')

df = pd.read_excel(excel_file)

# print Whole map from excel
print(df)

#parse reverse-row order
#ignore first/last row
#ignore first col
#rev row = (len(df[col])-1)-row

print(len(df), len(df.columns))

# Modified
for row in range(0,len(df)):
    # for row in reversed(range(0,len(df[col])-1)):
    for col in (range(0,len(df.columns)-1)):
    #for row in range(len(df[col])-2,0,-1):
        # print(row, col, df[col][row], sf[col][row])
        if(df[col][row] != 'x'):
            cell_type = "node"
            if(is_cell_station(sf[col][row])):
                cell_type = "station"
            print("    - !!python/tuple [" + str(col) +"," +  str((len(df)-1)-row)+"," + "\"" + str(df[col][row]) + "\"" +","+ "\"" + cell_type + "\"" +"]")


"""
Original  Code

for col in range(0,len(df)):
    for row in reversed(range(0,len(df[col])-1)):
    #for row in range(len(df[col])-2,0,-1):
        if(df[col][row] != 'x'):
            cell_type = "node"
            if(is_cell_station(sf[col][row])):
                cell_type = "station"
            print("    - !!python/tuple [" + str(col) +"," +  str((len(df)-2)-row)+"," + "\"" + str(df[col][row]) + "\"" +","+ "\"" + cell_type + "\"" +"]")
"""