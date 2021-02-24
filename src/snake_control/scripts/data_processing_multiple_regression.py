import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

np.random.seed(12345)
data = pd.read_csv("/home/christianluu/snake_ws/data/module7.csv")

x = data.iloc[:,:-1].values # x is the currents
y = data.iloc[:,-1].values # y is efforts to be predicted

from sklearn.model_selection import train_test_split
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = 0.2)

from sklearn.linear_model import LinearRegression
multi_regressor = LinearRegression()
multi_regressor.fit(x_train, y_train)
# y_pred = multi_regressor.predict(x_test)
# print(np.average(np.abs(y_test-y_pred)))
# print(y_pred[0])
# print(x_test[0])
# print(multi_regressor.predict([x_test[0]])[0])