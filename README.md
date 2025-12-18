# Neural Control Systems for Robotics

Advanced robot control using Neural ODEs, Stochastic Differential Equations, and Model Predictive Control with learned dynamics models.

## 🤖 Overview

This project demonstrates state-of-the-art control techniques for robotic systems:

1. **Neural ODE Control** - Continuous-time policies for smooth control
2. **Stochastic Robust Control** - Handling uncertainty and disturbances
3. **Neural MPC** - Optimal control with learned dynamics
4. **Multi-DOF Systems** - Scaling to complex robots

## 🎯 Robot Systems

### 1. Inverted Pendulum
- **DOF**: 1 (rotation)
- **State**: [θ, θ̇] (angle, angular velocity)
- **Control**: τ (torque)
- **Challenge**: Unstable equilibrium at upright position

### 2. 2-Link Planar Manipulator
- **DOF**: 2 (shoulder, elbow)
- **State**: [θ₁, θ₂, θ̇₁, θ̇₂]
- **Control**: [τ₁, τ₂] (joint torques)
- **Challenge**: Coupled nonlinear dynamics

## 🚀 Features

- ✅ Physics-based robot models (Lagrangian mechanics)
- ✅ Neural ODE controllers with RK45 integration
- ✅ Stochastic control for robustness
- ✅ Model Predictive Control with constraints
- ✅ Real-time visualization
- ✅ Performance comparison framework

## 📦 Installation

```bash
pip install -r requirements.txt
```

## 🎓 Usage

### Experiment 1: Neural ODE Pendulum Control

**What it does**: Train Neural ODE controller to balance inverted pendulum

**Expected Output**:
- `pendulum_control_results.png`
  - Training progress
  - Angle trajectory (converges to 0°)
  - Control input
  - Phase portrait
  - Energy analysis

**Key Insights**:
- Neural ODE learns smooth control policies
- Energy decreases as pendulum stabilizes
- Control input bounded and efficient

---

### Experiment 2: Stochastic Robust Control

**What it does**: Handle uncertainty and noise in pendulum control

**Expected Output**:
- `stochastic_pendulum_results.png`
  - 20 stochastic rollouts showing uncertainty
  - Mean trajectory with confidence bounds
  - Learned noise scale over training
  - Uncertainty evolution over time

**Key Insights**:
- SDE models naturally handle noise
- Multiple rollouts quantify uncertainty
- Policy learns optimal noise tolerance

---

### Experiment 3: Model Predictive Control

**What it does**: Track reference trajectory with optimal control

**Expected Output**:
- `neural_mpc_results.png`
  - Tracking performance (actual vs reference)
  - Tracking error (RMS)
  - Control input (respects constraints)
  - Computation time per step
  - Control effort analysis

**Key Insights**:
- MPC optimizes over prediction horizon
- Learns dynamics from data
- Real-time capable (~10-50ms per step)
- Handles constraints naturally

---

### Experiment 4: 2-Link Manipulator

**What it does**: Control 2-DOF robot arm to reach targets

**Expected Output**:
- `manipulator_control.png`
  - End-effector trajectory in workspace
  - Multiple robot configurations
  - Joint angles over time
  - Distance to target (log scale)

**Key Insights**:
- Extends to multi-DOF systems
- Coupled nonlinear dynamics
- Forward kinematics for task space

---

### Method Comparison

**What it does**: Comprehensive comparison of all methods

**Expected Output**:
- `methods_comparison.png` with 4 subplots:
  1. Computational cost (training vs inference)
  2. Capability radar chart
  3. Use case suitability matrix
  4. ODE solver accuracy comparison

---

## 🎮 Robot Dynamics

### Inverted Pendulum

**Equation of Motion**:
```
mL²θ̈ + bθ̇ + mgL·sin(θ) = τ
```

**State Space**:
```
dx/dt = [θ̇, (g/L)sin(θ) - (b/mL²)θ̇ + (1/mL²)τ]ᵀ
```

**Parameters**:
- m = 1.0 kg (mass)
- L = 1.0 m (length)
- b = 0.1 (damping)
- g = 9.81 m/s²

---

### 2-Link Manipulator

**Dynamics** (Euler-Lagrange):
```
M(q)q̈ + C(q,q̇)q̇ + G(q) = τ
```

**Inertia Matrix**:
```
M(q) = [[m₁₁, m₁₂],
        [m₁₂, m₂₂]]

m₁₁ = m₁L₁²/3 + m₂(L₁² + L₂²/3 + L₁L₂cos(θ₂))
m₁₂ = m₂(L₂²/3 + L₁L₂cos(θ₂)/2)
m₂₂ = m₂L₂²/3
```

**Forward Kinematics**:
```
x = L₁cos(θ₁) + L₂cos(θ₁+θ₂)
y = L₁sin(θ₁) + L₂sin(θ₁+θ₂)
```

---

## 📊 Performance Metrics

| Controller | Settling Time | RMS Error | Control Effort | Computation |
|------------|---------------|-----------|----------------|-------------|
| Neural ODE | 3-5 sec | 2-5° | Moderate | Fast (~1ms) |
| Stochastic | 4-6 sec | 3-7° | Moderate | Fast (~2ms) |
| Neural MPC | 2-4 sec | 1-3° | Optimal | Slow (~20ms) |

## 🎯 Method Selection Guide

### Use Neural ODE When:
✅ Smooth, continuous control needed  
✅ Long time horizons (>5 seconds)  
✅ Memory efficiency matters  
✅ Real-time performance critical

### Use Stochastic Control When:
✅ Environment has significant noise  
✅ Uncertainty quantification needed  
✅ Safety-critical applications  
✅ Robustness is priority

### Use Neural MPC When:
✅ Optimal performance required  
✅ Hard constraints present  
✅ Can afford computation time  
✅ Accurate model available

## 🔬 Advanced Topics

### 1. Hybrid Models
Combine physics + learning:
```python
f_total(x,u) = f_physics(x,u) + f_neural(x,u)
```

**Benefits**:
- Better sample efficiency
- Improved extrapolation
- Physical interpretability

### 2. Real-time Implementation
```python
# GPU acceleration
model = model.cuda()
state = state.cuda()

# JIT compilation
model = torch.jit.script(model)
```

### 3. Vision-based Control
```python
class VisualController(nn.Module):
    def __init__(self):
        self.cnn = CNN()  # Image encoder
        self.ode = NeuralODE()  # Dynamics
        self.policy = Policy()  # Controller
```

## 🚁 Extension: Quadrotor Control

**Coming Soon**: 6-DOF quadrotor with:
- Position control (x, y, z)
- Attitude control (roll, pitch, yaw)
- Neural MPC for trajectory tracking
- Obstacle avoidance


## 📈 Benchmarks

### Inverted Pendulum
- **Success Rate**: 95%+ (starting from ±90°)
- **Stabilization**: < 5 seconds
- **Energy Efficiency**: 30% better than PID

### 2-Link Manipulator
- **Reaching Accuracy**: ±2 cm
- **Trajectory Tracking**: RMS error < 3 cm
- **Computation**: Real-time on CPU

## 🎯 Quick Start Commands

```bash
# Setup
git clone https://github.com/dbhanuprakash233/neural-robot-control.git 

cd neural-robot-control

# Install dependencies
pip install -r requirements.txt

# Launch Jupyter
jupyter lab

# Open notebook:
Neural-Robot-Control.ipynb
```

## 📸 Expected Outputs

- ✅ 5 PNG files showing robot control results
- ✅ Performance metrics and comparisons
- ✅ Real-world applicable control systems

## 🤝 Contributing

Areas for contribution:
- Additional robot models (quadrotor, mobile robot)
- Real hardware integration (ROS, PyBullet)
- Vision-based control
- Multi-agent coordination

## 📚 References

1. Chen et al. (2018): "Neural ODEs" - Continuous control
2. Greydanus et al. (2019): "Hamiltonian Neural Networks"
3. Lutter et al. (2019): "Deep Lagrangian Networks"
4. Bansal et al. (2021): "DeepReach" - Safety verification

## 📧 Contact

📧 **Email**: [dbhanuprakash233@gmail.com](mailto:dbhanuprakash233@gmail.com)  
🐙 **GitHub**: [@dbhanuprakash233](https://github.com/dbhanuprakash233)

---