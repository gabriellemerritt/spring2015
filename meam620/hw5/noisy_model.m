function xdot = noisy_model(t, x, A, B, sigma, bias)

u = input_fun(t);
n = sigma^2 * randn(1) + bias;
xdot = A * x + B * (u + n);