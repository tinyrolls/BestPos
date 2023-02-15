# %%
import string
import plotly.express as px
import pandas as pd
import numpy as np
import plotly.graph_objects as go


orignal = "./result/filter.csv"
result1 = "./result/filter0.csv"
result2 = "./result/filter2.csv"
result3 = "./result/filter5.csv"

point_limit = 50

Data1 = pd.read_csv(orignal, header=0, nrows=point_limit)
Data2 = pd.read_csv(result1, header=0, nrows=point_limit)
Data3 = pd.read_csv(result2, header=0, nrows=point_limit)
Data4 = pd.read_csv(result3, header=0, nrows=point_limit)

oData = Data1.append(Data2).append(Data3).append(Data4)

fig = px.line_3d(oData, x='x_val', y='y_val', z='z_val', color='tag', markers=True)
# fig = px.scatter_3d(oData, x='x', y='y', z='z', color='tag')

fig.show()

