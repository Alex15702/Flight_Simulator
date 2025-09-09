# Flight Dynamics and Simulation Project

A comprehensive MATLAB-based flight simulation framework for aircraft dynamics analysis, featuring 3DOF and 6DOF flight models with trim analysis and visualization capabilities.

## Project Overview

This project implements flight dynamics simulation models for the Cessna Citation I aircraft, including:
- 3DOF and 6DOF flight dynamics models
- Trim condition analysis
- Free elevator dynamics
- Flight visualization with 3D aircraft models
- DATCOM aerodynamic data integration

## Project Structure

```
Flight_Simulator/
├── Datcom/                    # DATCOM aerodynamic data processing
├── Q3/                        # Question 3 implementation
├── Q7/                        # Question 7 - 6DOF flight dynamics
├── Q11/                       # Question 11 - Free elevator dynamics
├── Q16/                       # Question 16 implementation
├── Resources/                 # Shared resources and utilities
├── Trim Citation/             # Trim analysis for Citation aircraft
├── E6_5/                      # Exercise 6.5 implementation
└── Esercizio esame/          # Exam exercise materials
```

## Key Features

### Aircraft Models
- **Cessna Citation I**: Primary aircraft for simulation
- **HARV**: High Alpha Research Vehicle for comparison studies
- Support for custom aircraft configurations via text files

### Flight Dynamics Models
- **3DOF Model**: Simplified longitudinal dynamics
- **6DOF Model**: Full 6 degrees of freedom simulation
- **Free Elevator**: Stick-free elevator dynamics modeling
- **Coupled Dynamics**: Multi-axis coupling effects

### Aerodynamic Data Integration
- DATCOM coefficient import and processing
- Coefficient interpolation functions for:
  - [`CL`](Flight_Simulator/Trim Citation/CL.m) - Lift coefficient
  - [`CD`](Flight_Simulator/Trim Citation/CD.m) - Drag coefficient  
  - [`Cpitch`](Flight_Simulator/Trim Citation/Cpitch.m) - Pitching moment coefficient
  - [`Croll`](Flight_Simulator/Trim Citation/Croll.m) - Rolling moment coefficient

### Visualization Tools
- 3D aircraft trajectory plotting
- Real-time flight path visualization
- Body axes and Earth axes display
- Multiple aircraft shape models (Citation, MiG-29, PA24-250)

## Core Components

### Aircraft Class
The [`DSVAircraft`](Flight_Simulator/Trim Citation/DSVAircraft.m) class handles:
- Aircraft geometric properties
- Mass and inertia data
- Propulsion characteristics
- Data loading from configuration files

### Flight Dynamics Equations
Key simulation files:
- [`eqLongDynamicStickFixed_6DoF_Vabpqr_Euler.m`](Flight_Simulator/Q7/Virata Citation/eqLongDynamicStickFixed_6DoF_Vabpqr_Euler.m) - 6DOF stick-fixed dynamics
- [`eqLongDynamicFreeElevator_3DoF_Vabpqr_Euler.m`](Flight_Simulator/Q11/Modello3DOFStickFree/eqLongDynamicFreeElevator_3DoF_Vabpqr_Euler.m) - 3DOF free elevator dynamics
- [`eqLongDynamicFreeElevator_6DoF_Vabpqr_Euler.m`](Flight_Simulator/Q11/Modello6DOFAccoppiamento/eqLongDynamicFreeElevator_6DoF_Vabpqr_Euler.m) - 6DOF free elevator dynamics

### Utility Functions
- [`density.m`](Flight_Simulator/Q7/Virata Citation/density.m) - Atmospheric density calculation
- [`Chae.m`](Flight_Simulator/Q11/Modello6DOFAccoppiamento/Chae.m) - Elevator hinge moment coefficient
- Aircraft shape loading and display utilities

## Usage Examples

### Basic 6DOF Simulation
```matlab
% Load aircraft data
myAC = DSVAircraft('CITATION.txt');

% Set initial conditions
state_0 = [V0, alpha_B0, beta_der0, p0, q0, r0, xEG_0, yEG_0, zEG_0, phi0, theta0, psi0];

% Run simulation
[vTime, mState] = ode45(@eqLongDynamicStickFixed_6DoF_Vabpqr_Euler, [0 t_fin], state_0);
```

### Trim Analysis
```matlab
% Define trim conditions
trim_conditions = [alpha_B, delta_e, delta_a, delta_r, delta_T, beta];

% Find trim state
[trim_result] = fsolve(@trim_equations, initial_guess);
```

### Flight Visualization
```matlab
% Load aircraft shape
shape = loadAircraftMAT('aircraft_mig29.mat', shapeScaleFactor);

% Plot trajectory
plotTrajectoryAndBodyE(h_fig, shape, vXYZe, vEulerAngles, options);
```

## Aircraft Configuration Files

Aircraft parameters are defined in text files:
- [`CITATION.txt`](Flight_Simulator/Trim Citation/CITATION.txt) - Cessna Citation I data
- [`HARV.txt`](Flight_Simulator/Q7/Verifica condizioni di trim 6DOF/HARV.txt) - HARV aircraft data

## Dependencies

- MATLAB with Aerospace Toolbox
- Control System Toolbox (for trim analysis)
- Signal Processing Toolbox (for signal editor)

## Research Applications

This framework supports research in:
- Flight dynamics analysis
- Control system design
- Trim optimization
- Flight envelope analysis
- Aircraft handling qualities assessment

## Notes

- Inertia values for Citation aircraft are estimated based on HARV proportions
- DATCOM data includes downwash effects for elevator modeling
- Engine effects are included in thrust modeling
- All simulations use International Standard Atmosphere (ISA) conditions

## Contributing

When adding new aircraft models or simulation capabilities:
1. Follow the established file naming conventions
2. Use consistent variable naming (alpha_B, delta_e, etc.)
3. Include proper unit conversions using MATLAB's conversion functions
4. Document any assumptions or limitations in log files

## License

This project is an academic coursework for "Dinamica e Simulazione di Volo" (Flight Dynamics and Simulation) course.
