# NOTE THAT THIS IS WRITTEN IN PYTHON 3.7.8
# THE REST IS WRITTEN IN PYTHON 2.7

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

currents = pd.read_csv("/home/christianluu/snake_ws/data/data_r0.060_k1.3_amp_2.1/data_r0.060_k1.3_amp_2.1_currents.txt")
efforts = pd.read_csv("/home/christianluu/snake_ws/data/data_r0.060_k1.3_amp_2.1/data_r0.060_k1.3_amp_2.1_efforts.txt")
thetas = pd.read_csv("/home/christianluu/snake_ws/data/data_r0.060_k1.3_amp_2.1/data_r0.060_k1.3_amp_2.1_thetas.txt")
x1 = currents.iloc[:-2, 7:8].values # discard the last two rows (x2 doesn't have those 2 rows)
x2 = thetas.iloc[:, 7:8].values # only use the 7th module's data for now...
x = np.concatenate((x1,x2),axis=1)
y = efforts.iloc[:-2, 7:8].values


from sklearn.model_selection import train_test_split
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = 0.2) # test data is 20%

from sklearn.linear_model import LinearRegression
regressor = LinearRegression()
regressor.fit(x_train, y_train)

y_pred = regressor.predict(x_test)
print(y_test-y_pred)
