import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

# Learned the majority of sklearn, etc. from https://www.udemy.com/course/machinelearning/

np.random.seed(12345)

currents = pd.read_csv("/home/christianluu/snake_ws/data/compiled_currents.txt")
efforts = pd.read_csv("/home/christianluu/snake_ws/data/compiled_efforts.txt")
thetas = pd.read_csv("/home/christianluu/snake_ws/data/data_r0.060_k1.3_amp_2.1/data_r0.060_k1.3_amp_2.1_thetas.txt")
x = currents.iloc[:-2, 7:8].values # discard the last two rows (x2 doesn't have those 2 rows)
# x2 = thetas.iloc[:, 7:8].values # only use the 7th module's data for now...
# x = np.concatenate((x1,x2),axis=1)
y = efforts.iloc[:-2, 7:8].values


from sklearn.model_selection import train_test_split
x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = 0.1) # test data is 20%

# LINEAR REGRESSION
from sklearn.linear_model import LinearRegression
lin_regressor = LinearRegression()
lin_regressor.fit(x_train, y_train)

y_pred_lin = lin_regressor.predict(x_test)
# print(np.average(np.abs(y_test-y_pred_lin)))

# Visualize Linear Regression
plt.scatter(x_test, y_test, color = 'red')
plt.plot(x_test, lin_regressor.predict(x_test), color = 'blue')
plt.title('Linear Regression')
plt.xlabel('Current')
plt.ylabel('Effort')
# plt.show()

# POLYNOMIAL REGRESSION
from sklearn.preprocessing import PolynomialFeatures
poly_regressor = PolynomialFeatures(degree = 4)
x_poly = poly_regressor.fit_transform(x_train)
poly_regressor_2 = LinearRegression() # apply regression on degree 8 polynomial
poly_regressor_2.fit(x_poly, y_train)

y_pred_poly = poly_regressor_2.predict(poly_regressor.fit_transform(x_test))
# print(np.average(np.abs(y_test-y_pred_poly)))

y_pred_poly = poly_regressor_2.predict(poly_regressor.fit_transform([[0.5]]))
# print(y_pred_poly)

# Visualize Polynomial Regression
plt.scatter(x_test, y_test, color = 'red')
plt.plot(x_test, poly_regressor_2.predict(poly_regressor.fit_transform(x_test)), color = 'blue')
plt.title('Polynomial Regression')
plt.xlabel('Current')
plt.ylabel('Effort')
# plt.show()



# NEW DATA
data = pd.read_csv("/home/christianluu/snake_ws/data/module7.csv")
x = data.iloc[:,:-1].values # x is the currents
y = data.iloc[:,-1].values # y is efforts to be predicted

x_train, x_test, y_train, y_test = train_test_split(x, y, test_size = 0.2)

# MULTIPLE LINEAR REGRESSION
multi_lin_regressor = LinearRegression()
multi_lin_regressor.fit(x_train, y_train)