# EKF Implementation: Line-by-Line Explanation (MSc)

File modified: `b31ys_kalman/kalman_filter_template.py`

This document explains **every line that was added** to the template, in the
order it appears in the file. Line numbers refer to the modified file.

Nothing from the original template was deleted. A few original lines were
*disabled* by turning them into comments, never removed. These are the three
`raise NotImplementedError(...)` placeholders, the original `Q`, `R_gps` and
`R_odom` values, and `x_pred = None` / `P_pred = None`. Each one carries a
"kept for reference" note. Lines not listed below are original template code.

Notation: `x` is the state vector `[x, y, θ]ᵀ`, `P` is its 3×3 covariance,
`z` is a measurement, `H` is the measurement matrix, and `R` and `Q` are
noise covariances.

---

## Part A: Line-by-line walkthrough

### A1. Noise covariances in `__init__` (lines 39–82)

#### Process noise `Q` (lines 39–50)

| Line | Code | Explanation |
|---|---|---|
| 39 | `# Original template value (kept for reference):` | Comment saying that the next line is the template's original value. |
| 40 | `# self.Q = np.diag([0.02, 0.02, 0.01])` | The original line, commented out rather than deleted. |
| 41 | `#` | Blank comment line used as a separator. |
| 42–49 | `# DONE (MSc): Q is added once per prediction step ...` | Comment explaining what `Q` means and why this value was chosen (see Part B). |
| 50 | `self.Q = np.diag([0.002, 0.002, 0.001])` | Creates a 3×3 diagonal matrix. The entries are the variance added at every 0.1 s prediction to x (m²), y (m²) and θ (rad²). It models how much the real robot motion differs from the commanded `cmd_vel`. The off-diagonal entries are zero because the three errors are assumed independent. |

#### GPS noise `R_gps` (lines 54–60)

| Line | Code | Explanation |
|---|---|---|
| 54 | `# Original template value (kept for reference):` | Marks the original value. |
| 55 | `# self.R_gps = np.diag([0.25, 0.25])` | Original line, kept as a comment. |
| 56 | `#` | Separator. |
| 57–59 | `# DONE (MSc): fake_sensors.py adds Gaussian noise ...` | Comment giving the reason for the value: GPS σ = 0.15 m, so the variance is 0.0225 m². |
| 60 | `self.R_gps = np.diag([0.03, 0.03])` | 2×2 diagonal covariance of the GPS measurement `[x, y]`. The value 0.03 m² is the true variance rounded up slightly, so the filter does not over-trust GPS. |

#### Odometry noise `R_odom` (lines 64–78)

| Line | Code | Explanation |
|---|---|---|
| 64 | `# Original template value (kept for reference):` | Marks the original value. |
| 65 | `# self.R_odom = np.diag([1.0, 1.0, 0.4])` | Original line, kept as a comment. |
| 66 | `#` | Separator. |
| 67–77 | `# DONE (MSc): fake_sensors.py adds white noise ...` | Comment explaining why `R_odom` is much larger than the raw sensor noise (drift; see Part B). |
| 78 | `self.R_odom = np.diag([10.0, 10.0, 4.0])` | 3×3 diagonal covariance of the odometry measurement `[x, y, θ]`. The large values mean that one `/odom1` sample moves the estimate only slightly. This stops the odometry drift from pulling the estimate away from the GPS. |

#### Diagnostic counters (lines 80–82)

| Line | Code | Explanation |
|---|---|---|
| 80 | `# MSc: counters, used only for logging/diagnostics.` | Comment. |
| 81 | `self.gps_updates = 0` | Counts how many GPS corrections have been applied. It is printed in the log, so you can check GPS is being fused. |
| 82 | `self.odom_updates = 0` | Same as line 81, for odometry corrections. |

---

### A2. Prediction step in `update_kalman` (lines 192–216)

`dt` (the time elapsed since the last cycle, in seconds) has already been
computed by the template code above these lines.

| Line | Code | Explanation |
|---|---|---|
| 192 | `# Original template placeholders (kept for reference):` | Marks the disabled placeholders. |
| 193 | `# x_pred = None` | Original placeholder, commented out. Left active, it would have made the template raise `NotImplementedError`. |
| 194 | `# P_pred = None` | Same as line 193, for the covariance. |
| 196 | `# DONE (MSc): nonlinear state prediction f(x, u).` | Comment: the next lines compute the predicted state with the motion model `f`. |
| 197 | `theta = float(self.x[2, 0])` | Reads the current heading θ from the state vector (row 2, column 0) as a plain Python float. It is stored once because the prediction and the Jacobian must both use the **previous** heading. |
| 198 | `v = self.v` | Local copy of the commanded linear speed (m/s), stored by `cmd_callback`. A copy means the value cannot change half-way through this cycle and keeps the formulas short. |
| 199 | `omega = self.omega` | Local copy of the commanded turn rate ω (rad/s). |
| 201 | `x_pred = np.array([` | Starts building the predicted 3×1 column vector. |
| 202 | `[self.x[0, 0] + v * math.cos(theta) * dt],` | **x prediction:** `x + v·cos θ·dt`. The robot moves `v·dt` metres along its heading, and the x component of that move is `cos θ`. |
| 203 | `[self.x[1, 0] + v * math.sin(theta) * dt],` | **y prediction:** `y + v·sin θ·dt`. The y component of the same move. |
| 204 | `[wrap_angle(theta + omega * dt)]` | **θ prediction:** `θ + ω·dt`. The result is passed through `wrap_angle` (provided by the template) so θ stays in [-π, π). |
| 205 | `])` | Closes the array. `x_pred` has shape (3, 1), the same as `self.x`. |
| 207–208 | `# DONE (MSc): motion Jacobian F = df/dx ...` | Comment: the motion model is nonlinear (it uses sin and cos), so the EKF linearises it with its Jacobian. |
| 209 | `F = np.array([` | Starts the 3×3 Jacobian `F = ∂f/∂x`. |
| 210 | `[1.0, 0.0, -v * math.sin(theta) * dt],` | Row 1 = derivatives of `x_next`: ∂/∂x = 1, ∂/∂y = 0, ∂/∂θ = `−v·sin θ·dt` (the derivative of `cos θ` is `−sin θ`). |
| 211 | `[0.0, 1.0,  v * math.cos(theta) * dt],` | Row 2 = derivatives of `y_next`: 0, 1, `v·cos θ·dt` (the derivative of `sin θ` is `cos θ`). |
| 212 | `[0.0, 0.0,  1.0]` | Row 3 = derivatives of `θ_next`: θ depends only on itself. |
| 213 | `])` | Closes the Jacobian. Its third column links heading uncertainty to position uncertainty. This link is what allows the GPS, which measures position only, to correct the heading indirectly. |
| 215 | `# DONE (MSc): covariance propagation.` | Comment. |
| 216 | `P_pred = F @ self.P @ F.T + self.Q` | **EKF covariance prediction:** `P⁻ = F P Fᵀ + Q`. `@` is matrix multiplication. `F P Fᵀ` carries the existing uncertainty through the linearised motion, and `+ Q` adds the new uncertainty from the imperfect motion model. |

Lines 218–224 are original template code. The `None` check now passes, and
`self.x` and `self.P` are replaced by the predictions.

---

### A3. GPS correction (lines 247–262)

Lines 230–235 (original) have already checked that a new GPS message exists,
copied it into `z_gps` and set `self.gps_measurement = None`, so the same
fix is never used twice.

| Line | Code | Explanation |
|---|---|---|
| 247 | `# Original template placeholder (kept, disabled):` | Marks the disabled placeholder. |
| 248–250 | `# raise NotImplementedError(...)` | Original placeholder, commented out. Left active, it would crash the node on the first GPS message. |
| 252 | `# DONE (MSc): GPS observes x and y only.` | Comment. |
| 253 | `H_gps = np.array([` | Starts the 2×3 measurement matrix that maps the state to what the GPS measures. |
| 254 | `[1.0, 0.0, 0.0],` | The first GPS value is the state's x. |
| 255 | `[0.0, 1.0, 0.0]` | The second GPS value is the state's y. GPS gives no heading, so the θ column is zero. |
| 256 | `])` | Closes the matrix. |
| 258 | `# Innovation y = z - H x (no angle in it).` | Comment. |
| 259 | `innovation = z_gps - H_gps @ self.x` | **Innovation** (measurement residual): the measured position minus the position the filter expected. It has shape (2, 1). No angle is involved, so no wrapping is needed. |
| 261 | `self.ekf_correct(innovation, H_gps, self.R_gps)` | Calls the shared update method (section A6) with the GPS-specific `H` and `R`. That method updates `self.x` and `self.P`. |
| 262 | `self.gps_updates += 1` | Adds one to the GPS counter for the log. |

---

### A4. Odometry correction, MSc only (lines 284–300)

Lines 268–274 (original) run only when `level == 'msc'` and a new `/odom1`
message has arrived. They copy it into `z_odom` and clear the stored value,
so each odometry message is used once.

| Line | Code | Explanation |
|---|---|---|
| 284 | `# Original template placeholder (kept, disabled):` | Marks the disabled placeholder. |
| 285–287 | `# raise NotImplementedError(...)` | Original placeholder, commented out. |
| 289 | `# DONE (MSc): odometry observes the full state.` | Comment. |
| 290 | `H_odom = np.eye(3)` | 3×3 identity matrix. Odometry measures x, y and θ directly, so the expected measurement is the state itself. |
| 292 | `innovation = z_odom - H_odom @ self.x` | Odometry innovation, shape (3, 1): measured `[x, y, θ]` minus the estimated `[x, y, θ]`. |
| 294–296 | `# Heading innovation must lie in [-pi, pi) ...` | Comment explaining why the next line is needed. |
| 297 | `innovation[2, 0] = wrap_angle(innovation[2, 0])` | **Angle wrapping of the heading error.** If the measurement is +3.13 rad and the estimate is −3.13 rad, the raw difference is 6.26 rad, but the true difference is about −0.02 rad. Without this line the filter would turn its heading estimate almost a full circle the wrong way whenever the robot crosses ±π. |
| 299 | `self.ekf_correct(innovation, H_odom, self.R_odom)` | Applies the shared EKF update with the odometry `H` and `R`. It starts from the state already corrected by GPS (if a GPS fix arrived this cycle). This is sequential sensor fusion. |
| 300 | `self.odom_updates += 1` | Adds one to the odometry counter for the log. |

---

### A5. Diagnostic log (lines 302–309)

| Line | Code | Explanation |
|---|---|---|
| 302 | `# MSc: diagnostics, printed at most every 5 s.` | Comment. |
| 303 | `self.get_logger().info(` | Calls the ROS 2 logger at INFO level, so the message appears in the launch terminal. |
| 304 | `f'x={self.x[0, 0]:.2f} y={self.x[1, 0]:.2f} '` | Formats the estimated x and y to 2 decimal places. |
| 305 | `f'theta={self.x[2, 0]:.2f} \| '` | Adds the estimated heading. |
| 306 | `f'GPS updates={self.gps_updates} '` | Adds the GPS update count. |
| 307 | `f'odom updates={self.odom_updates}',` | Adds the odometry update count. If either count stays at 0, that sensor is not being received. |
| 308 | `throttle_duration_sec=5.0` | ROS 2 option that prints the message at most once every 5 s, instead of 10 times per second. |
| 309 | `)` | Closes the call. |

Line 311 (`self.publish_estimate()`) is original.

---

### A6. New method `ekf_correct` (lines 313–339)

A single update method is shared by both sensors, so the EKF equations are
written only once. Only `innovation`, `H` and `R` change between GPS and
odometry.

| Line | Code | Explanation |
|---|---|---|
| 313 | `def ekf_correct(self, innovation, H, R):` | Defines the method. Its inputs are the innovation `(m×1)`, the measurement matrix `H (m×3)` and the noise covariance `R (m×m)`, where `m` = 2 for GPS and 3 for odometry. |
| 314–321 | `"""..."""` | Docstring describing the method's purpose and the shape of each argument. |
| 323 | `# Innovation covariance.` | Comment. |
| 324 | `S = H @ self.P @ H.T + R` | **Innovation covariance** `S = H P Hᵀ + R`. It is the expected spread of the innovation: the state uncertainty seen through the sensor, plus the sensor noise. |
| 326 | `# Kalman gain (solve is more stable than inv(S)).` | Comment. |
| 327 | `K = np.linalg.solve(S.T, (self.P @ H.T).T).T` | **Kalman gain** `K = P Hᵀ S⁻¹`. Instead of inverting `S`, it solves the linear system `S Kᵀ = (P Hᵀ)ᵀ` and transposes the result. This gives the same `K` and is more accurate numerically. `K` (3×m) decides how much of the innovation moves each state variable. |
| 329 | `# State correction.` | Comment. |
| 330 | `self.x = self.x + K @ innovation` | **State update** `x = x + K·innovation`. The estimate moves toward the measurement, weighted by the gain. |
| 331 | `self.x[2, 0] = wrap_angle(self.x[2, 0])` | The update may push θ outside [-π, π), so it is wrapped back. |
| 333–336 | `# Covariance correction, Joseph form: ...` | Comment giving the formula and why it is used. |
| 337 | `I_KH = np.eye(3) - K @ H` | Computes `(I − K H)` once, because the next line uses it twice. |
| 338 | `self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T` | **Covariance update, Joseph form:** `P = (I−KH) P (I−KH)ᵀ + K R Kᵀ`. Mathematically it equals the short form `(I−KH)P`, but it stays symmetric and positive definite even with rounding errors, so the filter cannot become numerically unstable. |
| 339 | `self.P = 0.5 * (self.P + self.P.T)` | Averages `P` with its transpose to remove any tiny asymmetry left by floating-point rounding. A covariance matrix must be symmetric. |

---

### A7. Publishing the covariance (lines 356–364)

| Line | Code | Explanation |
|---|---|---|
| 356–358 | `# MSc: publish the EKF covariance ...` | Comment. The ROS `Odometry` pose covariance is a 6×6 matrix over `[x, y, z, roll, pitch, yaw]`, stored as a flat list of 36 numbers. |
| 359 | `cov = np.zeros((6, 6))` | Creates a 6×6 matrix of zeros. z, roll and pitch are not estimated, so they stay 0. |
| 360 | `idx = [0, 1, 5]` | Where each EKF state goes in the 6×6 matrix: x → 0, y → 1, θ (yaw) → 5. |
| 361 | `for i in range(3):` | Loops over the rows of the 3×3 `P`. |
| 362 | `for j in range(3):` | Loops over the columns of the 3×3 `P`. |
| 363 | `cov[idx[i], idx[j]] = self.P[i, j]` | Copies each entry of `P` to its position in the 6×6 matrix, including the cross terms (for example x–yaw goes to `cov[0, 5]`). |
| 364 | `msg.pose.covariance = [float(c) for c in cov.flatten()]` | Flattens the matrix row by row into 36 numbers and converts each to a Python float, as the ROS message field requires. RViz can then draw the uncertainty ellipse on `/kalman/odom`. |

Line 366 (`self.publisher.publish(msg)`) is original.

---

## Part B: Background and justification

### B1. How the filter meets the MSc requirements

| # | Requirement | Lines |
|---|---|---|
| 1 | Predict `[x, y, θ]` from `/cmd_vel` | 197–205 |
| 2 | Propagate covariance with the motion Jacobian | 209–216 |
| 3 | Correct with `/gps` | 253–262, 313–339 |
| 4 | Additional correction with `/odom1` | 290–300, 313–339 |
| 5 | Tune `Q` | 50 |
| 6 | Tune `R_gps`, `R_odom` | 60, 78 |
| — | Handle different measurement rates; use each measurement once | original 232–235 and 273–274 (copy, then set to `None`) |
| — | Handle angular differences | 204, 297, 331 |

Order of operations in each 0.1 s cycle:

```
predict (cmd_vel) → GPS update (if new) → odom1 update (if new) → log → publish
```

### B2. Why `R_odom` is much larger than the raw sensor noise

`fake_sensors.py` builds `/odom1` as:

```
z = true_state + drift + white_noise
white_noise ~ N(0, σ = [0.35, 0.35, 0.10])
drift      += N(0, 0.025·√dt)    # random walk, grows over time
```

Plugging in the white-noise variance (0.1225, 0.1225, 0.01) would be wrong,
for three reasons:

1. The drift is a random walk and is not part of the 3-element state, so the
   EKF cannot estimate or remove it. After 10 minutes it reaches roughly
   0.6 m in position and 0.6 rad in heading.
2. The errors are correlated in time, but the EKF assumes each measurement
   has independent noise. About 10 odometry samples are fused per GPS fix,
   so the filter would become far too confident in the drifting odometry.
3. GPS has no drift, so it must stay the absolute position reference.

### B3. How the values were chosen (offline sweep)

ROS 2 was not available on the development machine. The node was run
unchanged with stubbed ROS modules, against a simulator that reproduces
`fake_sensors.py` (same σ values, same drift, GPS at 1 Hz). The test assumed
a 5 % speed error and a 0.02 rad/s turn-rate error between commanded and
actual motion. Each run lasted 600 s; RMSE (root-mean-square error) is
measured against the true trajectory.

| Q | R_odom | Position RMSE | Heading RMSE |
|---|---|---|---|
| [0.005,0.005,0.002] | [0.6,0.6,0.05] (≈ raw sensor noise) | 0.16 – 0.26 m | 0.14 – 0.21 rad |
| [0.002,0.002,0.001] | [2,2,0.5] | 0.13 – 0.15 m | 0.12 – 0.19 rad |
| [0.002,0.002,0.001] | **[10,10,4] (chosen)** | **0.13 – 0.14 m** | **0.07 – 0.11 rad** |
| [0.002,0.002,0.001] | ∞ (GPS only, for comparison) | 0.14 m | 0.06 – 0.07 rad |

For reference, raw GPS RMSE was about 0.21 m and raw `/odom1` RMSE was about
0.60 m. The EKF is therefore about 35 % better than GPS alone and about 4×
better than the noisy odometry.

> These values are a starting point. The real Gazebo robot may follow
> `cmd_vel` differently from the offline model. If the estimate lags on
> turns, increase `Q`. If it is jittery, decrease `Q`. If it drifts away
> from the GPS points, increase `R_odom`.

### B4. How to run

```bash
cd ~/ros2_ws
colcon build --packages-select b31ys_kalman
source install/setup.bash
ros2 launch b31ys_kalman kalman_execution.launch.py student_name:=YourName level:=msc
```

In RViz (fixed frame `odom`), `/kalman/odom` should follow the `/odom`
reference closely. It should stay near the GPS points and not follow the
drift of `/odom1`. Both update counters in the terminal should increase.
