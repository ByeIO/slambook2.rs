import numpy as np
import scipy.optimize as opt
import matplotlib.pyplot as plt
import time

# 定义曲线模型
def curve_model(params, x):
    a, b, c = params
    return np.exp(a * x**2 + b * x + c)

# 定义误差函数
def error_function(params, x_data, y_data):
    return y_data - curve_model(params, x_data)

# 生成数据
def generate_data(a_true, b_true, c_true, N, w_sigma):
    np.random.seed(0)
    x_data = np.linspace(0, 1, N)
    y_data = curve_model([a_true, b_true, c_true], x_data) + np.random.normal(0, w_sigma, N)
    return x_data, y_data

# 主函数
def main():
    # 真实参数值
    a_true, b_true, c_true = 1.0, 2.0, 1.0
    # 估计参数值
    ae, be, ce = 2.0, -1.0, 5.0
    # 数据点数量
    N = 100
    # 噪声Sigma值
    w_sigma = 1.0

    # 生成数据
    x_data, y_data = generate_data(a_true, b_true, c_true, N, w_sigma)

    # 初始参数估计
    params_initial = np.array([ae, be, ce])

    # 优化
    print("Start optimization")
    start_time = time.time()
    result = opt.least_squares(error_function, params_initial, args=(x_data, y_data), method='lm')
    end_time = time.time()
    print(f"Solve time cost = {end_time - start_time} seconds.")

    # 输出优化值
    a_est, b_est, c_est = result.x
    print(f"Estimated model: a = {a_est}, b = {b_est}, c = {c_est}")

    # 可视化结果
    plt.scatter(x_data, y_data, label='Data points')
    x_fit = np.linspace(0, 1, 100)
    y_fit = curve_model([a_est, b_est, c_est], x_fit)
    plt.plot(x_fit, y_fit, 'r', label='Fitted curve')
    plt.legend()
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Curve Fitting')
    plt.show()

if __name__ == "__main__":
    main()