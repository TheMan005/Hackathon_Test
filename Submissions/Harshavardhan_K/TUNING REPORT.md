## Methodology

We used the **Ziegler–Nichols tuning method** to determine appropriate gains for the TurtleBot3 PID controller. The process involved:

- Disabling integral and derivative terms (`Ki = Kd = 0`)
- Incrementally increasing `Kp` until the robot exhibited consistent oscillations
- Measuring the oscillation period `Tu` for one full cycle
- Applying Ziegler-Nichols tuning formulas to calculate `Kp`, `Ki`, `Kd`



## 2. Final Gain Values

### Linear PID

- Ku = `___`
- Tu = `___ s`
- Kp = `___`
- Ki = `___`
- Kd = `___`

### Angular PID

- Ku = `___`
- Tu = `___ s`
- Kp = `___`
- Ki = `___`
- Kd = `___`

## 3. Performance Analysis

| Metric             | Linear     | Angular    | Target      |
|--------------------|------------|------------|-------------|
| Rise Time          | `___ s`    | `___ s`    | < 3.0 s     |
| Overshoot          | `___ %`    | `___ %`    | < 10%       |
| Settling Time      | `___ s`    | `___ s`    | < 5.0 s     |
| Steady-State Error | `___ cm`   | `___ °`    | < 5cm / 3°  |

## 4. Visualization

- `position_tracking.png` — shows trajectory
- `error_evolution.png` — shows PID error over time

## 5. Discussion

- **Challenges Faced:** e.g., overshoot in angular motion, delay in response, drift in simulation
- **Potential Improvements:** try Cohen-Coon, adaptive PID, or incorporate feedforward terms
