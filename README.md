# TSP Path Planner

A complete path planning solution combining the Traveling Salesman Problem (TSP) optimization with A* shortest path algorithm for grid-based navigation.

![Python Version](https://img.shields.io/badge/python-3.8%2B-blue)
![License](https://img.shields.io/badge/license-MIT-green)
![Status](https://img.shields.io/badge/status-beta-yellow)

## 🌟 Features

- **Complete Planning Pipeline**: Integrates TSP optimization with A* pathfinding
- **Optimal Waypoint Ordering**: Uses OR-Tools to solve TSP for minimal path cost
- **Collision-Free Paths**: A* algorithm ensures obstacle avoidance
- **Save/Load Solutions**: Persist planning results in NPZ or pickle format
- **Rich Visualization**: Static plots and animated robot navigation
- **Performance**: C++ TSP solver with Python fallback
- **Well-Tested**: Comprehensive unit tests with pytest

## 📋 Table of Contents

- [Installation](#installation)
- [Quick Start](#quick-start)
- [API Documentation](#api-documentation)
- [Examples](#examples)
- [Visualization](#visualization)
- [Performance](#performance)
- [Development](#development)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## 🚀 Installation

### Prerequisites

- Python 3.8 or higher
- C++ compiler (for building OR-Tools extension)
- OR-Tools library

### Step 1: Clone the Repository

```bash
git clone https://github.com/vaithak/TSP-all-pair-shortest.git
cd TSP-all-pair-shortest
```
