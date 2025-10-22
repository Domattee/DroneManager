import numpy as np
from scipy.interpolate import CubicSpline

# Algorythem written by Dr. techn. Babak Salamat, OOP structure created by Tim Drouven
# Date: 08.05.2024
# Update: Class. 23.05.2024

class GMP3Config:
    def __init__(self, maxit, alpha, wdamp, delta, vx_max, vy_max, Q11, Q22, Q12, dt, x_max, y_max, obstacles=None):
        """
        Initializes the configuration for the GMP3 algorithm using the GMP3Config class..
        Obstacles (list of tuples): List of coordinates (x, y, r)
        representing obstacles, if any and

        """
        self.maxit = maxit
        self.alpha = alpha
        self.wdamp = wdamp
        self.delta = delta
        # self.nobs   = nobs is now calculate by len(obstacles)
        # self.x_in   = x_in
        # self.y_in   = y_in
        # self.x_f1   = x_f1
        # self.y_f1   = y_f1
        # self.x_f2   = x_f2
        # self.y_f2   = y_f2
        self.vx_max = vx_max
        self.vy_max = vy_max
        self.Q11 = Q11
        self.Q22 = Q22
        self.Q12 = Q12
        self.dt = dt
        self.nobs = len(obstacles)
        self.obstacles = obstacles if obstacles is not None else []
        self.pathfound = []
        self.iterations_needed = []
        self.x_max = x_max
        self.y_max = y_max


class GMP3:
    def __init__(self, gmpConfig):
        """
        Initialize the GMP3 algorithm using the gmpConfig class.
        Obstacles (list of tuples): List of coordinates (x, y) representing obstacles, if any.
        """
        self.maxit = gmpConfig.maxit
        self.alpha = gmpConfig.alpha
        self.wdamp = gmpConfig.wdamp
        self.delta = gmpConfig.delta
        self.vx_max = gmpConfig.vx_max
        self.vy_max = gmpConfig.vy_max
        self.Q11 = gmpConfig.Q11
        self.Q22 = gmpConfig.Q22
        self.Q12 = gmpConfig.Q12
        self.dt = gmpConfig.dt
        self.tf = None
        self.x_max = gmpConfig.x_max
        self.y_max = gmpConfig.y_max
        self.nobs = gmpConfig.nobs
        self.obstacles = gmpConfig.obstacles if gmpConfig.obstacles is not None else []

        # these values will be set by calling the calculate method
        self.x_in = None
        self.y_in = None
        self.x_f1 = None
        self.y_f1 = None
        self.x_f2 = None
        self.y_f2 = None

        self.theta = []
        self.violation = []
        self.computedValue = []
        self.robs = []
        self.Xobs = []
        self.Yobs = []
        self.x = []
        self.y = []
        self.t = []
        self.xdot = []
        self.ydot = []
        self.verbose = False

    def enforce_bounds(self, x, y):
        """Ensures trajectory remains within both positive and negative bounds"""
        x = np.clip(x, -self.x_max, self.x_max)
        y = np.clip(y, -self.y_max, self.y_max)
        return x, y

    def Cost1(self, xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta):
        # Unpack theta into wx and wy
        half_n = len(theta) // 2
        wx = theta[:half_n]
        wy = theta[half_n:]
        # Time parameters
        Ts = self.dt
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)
        tt = np.linspace(0, tf, len(wx) + 2)
        # Interpolating the desired setpoints

        XS = np.array([x_in] + list(wx) + [x_f2])
        YS = np.array([y_in] + list(wy) + [y_f2])
        cs_x = CubicSpline(tt, XS, bc_type='natural')
        cs_y = CubicSpline(tt, YS, bc_type='natural')
        rxd = cs_x(t)
        ryd = cs_y(t)

        # Enforce boundary constraints
        rxd, ryd = self.enforce_bounds(rxd, ryd)

        # Velocity calculation
        vrxd = cs_x(t, 1)  # First derivative with respect to t
        vryd = cs_y(t, 1)
        vrxd = np.append(vrxd, 0)  # Append zero to maintain array size
        vryd = np.append(vryd, 0)

        # Define maximum velocities and control inputs
        vx_max = self.vx_max
        vy_max = self.vy_max
        Ux = Uy = 2.5

        # Initialize state and control input arrays
        x = x_in * np.ones(k)
        y = y_in * np.ones(k)
        xdot = np.zeros(k)
        ydot = np.zeros(k)

        # Calculate x, y, xdot, ydot
        for i in range(1, k):
            x[i] = x[i - 1] + vrxd[i - 1] * Ts
            y[i] = y[i - 1] + vryd[i - 1] * Ts
            xdot[i] = vrxd[i - 1]
            ydot[i] = vryd[i - 1]

        # Evaluate trajectory's performance
        QQ = np.array([[self.Q11, self.Q12], [self.Q12, self.Q22]])
        quad = np.sum(QQ[0, 0] * np.diff(x) ** 2 + QQ[1, 1] * np.diff(y) ** 2 + 2 * QQ[0, 1] * np.diff(x) * np.diff(y))
        # Obstacle violation measure
        nobs = len(xobs)
        Violation = 0
        for j in range(nobs):
            d = np.sqrt(
                QQ[0, 0] * (x - xobs[j]) ** 2 + QQ[1, 1] * (y - yobs[j]) ** 2 + QQ[0, 1] * (x - xobs[j]) ** 2 * (
                            y - yobs[j]) ** 2)
            v = np.maximum(1 - d / (robs[j] + 0.2), 0)

            vvx = np.maximum((xdot / (self.vx_max)) - 1, 0)
            vvy = np.maximum((ydot / (self.vy_max)) - 1, 0)

            Violation += 1.5 * np.mean(v) + 1.5 * np.mean(vvx) + 1.1 * np.mean(vvy) + 0.06 * np.std(v)
        J = - (0.5 * quad * (1 + 15 * Violation))
        return J

    def Cost(self, xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta):
        # Unpack theta into wx and wy
        half_n = len(theta) // 2
        wx = theta[:half_n]
        wy = theta[half_n:]
        # Time parameters
        Ts = self.dt
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)
        tt = np.linspace(0, tf, len(wx) + 2)
        # Interpolating the desired setpoints

        XS = np.array([x_in] + list(wx) + [x_f2])
        YS = np.array([y_in] + list(wy) + [y_f2])
        cs_x = CubicSpline(tt, XS, bc_type='natural')
        cs_y = CubicSpline(tt, YS, bc_type='natural')
        rxd = cs_x(t)
        ryd = cs_y(t)

        # Velocity calculation
        vrxd = cs_x(t, 1)  # First derivative with respect to t
        vryd = cs_y(t, 1)
        vrxd = np.append(vrxd, 0)  # Append zero to maintain array size
        vryd = np.append(vryd, 0)

        # Define maximum velocities and control inputs
        vx_max = self.vx_max
        vy_max = self.vy_max
        Ux = Uy = 2.5

        # Initialize state and control input arrays
        x = x_in * np.ones(k)
        y = y_in * np.ones(k)
        xdot = np.zeros(k)
        ydot = np.zeros(k)

        # Calculate x, y, xdot, ydot
        for i in range(1, k):
            x[i] = x[i - 1] + vrxd[i - 1] * Ts
            y[i] = y[i - 1] + vryd[i - 1] * Ts
            xdot[i] = vrxd[i - 1]
            ydot[i] = vryd[i - 1]

        # Evaluate trajectory's performance
        QQ = np.array([[self.Q11, self.Q12], [self.Q12, self.Q22]])
        quad = np.sum(QQ[0, 0] * np.diff(x) ** 2 + QQ[1, 1] * np.diff(y) ** 2 + 2 * QQ[0, 1] * np.diff(x) * np.diff(y))
        # Obstacle violation measure
        nobs = len(xobs)
        Violation = 0
        for j in range(nobs):
            d = np.sqrt(
                QQ[0, 0] * (x - xobs[j]) ** 2 + QQ[1, 1] * (y - yobs[j]) ** 2 + QQ[0, 1] * (x - xobs[j]) ** 2 * (
                            y - yobs[j]) ** 2)
            v = np.maximum(1 - d / (robs[j] + 0.2), 0)

            vvx = np.maximum((xdot / (self.vx_max)) - 1, 0)
            vvy = np.maximum((ydot / (self.vy_max)) - 1, 0)

            Violation += 1.5 * np.mean(v) + 1.5 * np.mean(vvx) + 1.1 * np.mean(vvy) + 0.06 * np.std(v)
        J = - (0.5 * quad * (1 + 15 * Violation))
        return J, Violation

    def Agents(self, xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta, delta, n):
        unit = np.zeros(n)
        grad = np.zeros(n)

        for i in range(n):
            unit[i] = 1
            grad[i] = (-self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta + 2 * delta * unit) +
                       8 * self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta + delta * unit) -
                       8 * self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta - delta * unit) +
                       self.Cost1(xobs, yobs, robs, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta - 2 * delta * unit)) / (
                                  12 * delta)
            # unit[i] = 0  # Reset the unit vector

        return grad

    def results(self, x_in, y_in, x_f1, x_f2, y_f1, y_f2, theta):
        half_n = len(theta) // 2
        wx = theta[:half_n]
        wy = theta[half_n:]

        # vx_max = self.vx_max
        # vy_max = self.vy_max
        # Ux = Uy = 2.5

        # Time parameters
        Ts = self.dt
        tf = self.tf
        t = np.arange(0, tf + Ts, Ts)
        k = len(t)

        tt = np.linspace(0, tf, len(wx) + 2)

        XS = np.array([x_in] + list(wx) + [x_f2])
        YS = np.array([y_in] + list(wy) + [y_f2])
        cs_x = CubicSpline(tt, XS, bc_type='natural')
        cs_y = CubicSpline(tt, YS, bc_type='natural')
        rxd = cs_x(t)
        ryd = cs_y(t)

        # Velocity calculation
        vrxd = cs_x(t, 1)  # First derivative with respect to t
        vryd = cs_y(t, 1)
        vrxd = np.append(vrxd, 0)  # Append zero to maintain array size
        vryd = np.append(vryd, 0)

        # Define maximum velocities and control inputs
        # vx_max = self.vx_max
        # vy_max = self.vy_max
        # Ux = Uy = 2.5

        # Initialize state and control input arrays
        x = x_in * np.ones(k)
        y = y_in * np.ones(k)
        xdot = np.zeros(k)
        ydot = np.zeros(k)
        vvx = np.zeros(k)
        vvy = np.zeros(k)

        # Calculate x, y, xdot, ydot
        for i in range(1, k):
            x[i] = x[i - 1] + vrxd[i - 1] * Ts
            y[i] = y[i - 1] + vryd[i - 1] * Ts
            xdot[i] = vrxd[i - 1]
            ydot[i] = vryd[i - 1]

        return t, x, y, xdot, ydot

    def calculate(self, startposition, finalposition):
        """
        Calculate the path using the start position [x: float, y: flaot] to the
        end/final position [x: float, y: float]
        """
        self.x_in = startposition[0]
        self.y_in = startposition[1]
        self.x_f1 = finalposition[0]
        self.y_f1 = finalposition[1]
        self.x_f2 = finalposition[0]
        self.y_f2 = finalposition[1]
        self.tf = ((np.sqrt((self.x_f2 - self.x_in) ** 2 + (self.y_f2 - self.y_in) ** 2)) / (
            np.sqrt((self.vx_max) ** 2 + (self.vy_max) ** 2))) * 2

        theta = np.zeros((2 * self.nobs, self.maxit))
        x_values = np.linspace(self.x_in, self.x_f2, self.nobs)
        y_values = np.linspace(self.y_in, self.y_f2, self.nobs)
        theta[:, 0] = np.concatenate([x_values, y_values])

        # Assuming necessary functions and config object are defined
        jj = np.zeros((1, self.maxit))
        previous_value = float('inf')  # Initialize previous value to infinity
        tolerance = 0.05  # Define a small tolerance value for the stopping criterion

        # Predefine some values / Reset the object
        self.pathfound = False
        self.iterations_needed = None
        self.x = None
        self.y = None

        self.theta = None
        self.robs = None
        self.Xobs = None
        self.Yobs = None
        self.xdot = None
        self.ydot = None

        theta_opt = []
        # value = 0
        V = 0
        # computedValue = 0
        m = 0
        v = 0
        beta1 = 0.01
        beta2 = 0.99

        # Initialize variables for stopping criteria
        previous_value = float('inf')
        consecutive_no_improvement = 0
        max_consecutive_no_improvement = 25  # Number of iterations to wait for improvement
        gradient_tolerance = 1e-5  # Tolerance for gradient norm

        for i in range(self.maxit - 1):

            obstacles = np.transpose(np.array(self.obstacles))

            Xobs = obstacles[0]
            Yobs = obstacles[1]
            robs = obstacles[2]

            grad = self.Agents(Xobs, Yobs, robs, self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                               theta[:, i], self.delta, 2 * self.nobs)

            # Dampen the learning rate
            self.alpha *= self.wdamp

            m = beta1 * m + (1 - beta1) * (grad);
            v = beta2 * v + (1 - beta2) * (grad) ** 2;

            mhat = m / (1 - beta1 ** i + 1);
            vhat = v / (1 - beta2 ** i + 1);

            # Update theta[:, i+1]
            theta[:, i + 1] = theta[:, i] + (self.alpha * mhat / (np.sqrt(1 + vhat)));
            # theta[:, i+1] = theta[:, i] + self.alpha * grad

            # Ensure theta values are clamped between x_in and x_f2
            # theta[:, i+1] = np.clip(theta[:, i+1], self.x_in, self.x_f2)

            # Calculate cost and violation for the new theta values at i+1
            jj, V = self.Cost(Xobs, Yobs, robs, self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                              theta[:, i + 1])

            # Normalize the value
            current_value = abs(jj)

            if (V == 0 and abs(current_value - previous_value) < tolerance):
                theta_opt = theta[:, i + 1]
                t, x, y, xdot, ydot = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                                                   theta_opt)
                self.pathfound = True
                break  # Exit the loop if optimal conditions are met

            # Check if the gradient norm is below the tolerance
            if np.linalg.norm(grad) < gradient_tolerance:
                theta_opt = theta[:, i + 1]
                t, x, y, xdot, ydot, ux, uy = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1,
                                                           self.y_f2, theta_opt)
                self.pathfound = True
                break  # Exit the loop if gradient norm is very small

            # Update previous value and check for consecutive no improvement
            if abs(current_value - previous_value) < tolerance:
                consecutive_no_improvement += 1
                if consecutive_no_improvement >= max_consecutive_no_improvement:
                    theta_opt = theta[:, i + 1]
                    t, x, y, xdot, ydot = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                                                       theta_opt)
                    self.pathfound = True
                    break  # Exit the loop if no improvement over many iterations
            else:
                consecutive_no_improvement = 0
            # Update previous value
            previous_value = current_value

            # value_str = f"{value:.4f}" if np.isscalar(value) else str(value)
            if self.verbose == True:
                print(f"Iteration {i + 1}   Violation = {V:.4f}   Computed Value = {current_value}")

        self.iterations_needed = i
        self.violation = V
        self.computedValue = current_value

        # Check if theta_opt was set and handle accordingly
        if theta_opt is not None:
            if self.verbose == True:
                print("Optimal theta found.")
            # Optional: do something with t, x, y, or theta_opt
        else:
            if self.verbose == True:
                print("No optimal theta found within the given iterations.")

        t, x, y, xdot, ydot = self.results(self.x_in, self.y_in, self.x_f1, self.x_f2, self.y_f1, self.y_f2,
                                           theta[:, i])

        self.x = x
        self.y = y
        self.t = t

        self.theta = theta
        self.robs = robs
        self.Xobs = Xobs
        self.Yobs = Yobs
        self.xdot = xdot
        self.ydot = ydot
        print(len(self.x))
        # return t, x, y, xdot, ydot
