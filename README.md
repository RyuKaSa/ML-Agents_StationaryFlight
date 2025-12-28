# Drone Flight Agent

A Unity ML-Agents project training a quadcopter drone to achieve stable stationary flight.

![Demo](Video.gif)

## Overview

The agent learns to control four independent motor thrusts to hover in place. It receives observations from velocity sensors, orientation data, and 14 raycasts for spatial awareness.

## Reward Structure

- Staying upright (positive)
- Minimizing velocity and angular velocity (negative penalty)
- Maintaining position near spawn point (negative penalty for drift)
- Survival time bonus

## Scripts

- **StationaryFlightAgent.cs** - ML-Agents agent handling observations, actions, and rewards
- **DronePhysics.cs** - Physics simulation applying thrust forces at motor positions

## Requirements

- Unity 2021.3+
- ML-Agents package (Python 3.9)