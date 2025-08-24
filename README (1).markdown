# Flight Route Optimizer

A C++ program to find the three fastest flight routes between airports using Dijkstra's algorithm.

## Overview
This program finds the three fastest flight routes between two airports based on IATA codes. It uses a dataset of global airports and routes, applying Dijkstra's algorithm with weighted edges for distance to compute distinct shortest paths.

## Requirements
- C++11 compatible compiler (e.g., MinGW-w64 GCC 15.2.0 or later)
- Data files: `data/global_airports.csv`, `data/global_routes.csv` (from https://github.com/jpatokal/openflights)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/Flight-Route-Optimizer.git
   cd Flight-Route-Optimizer
   ```
2. Ensure `data/global_airports.csv` and `data/global_routes.csv` are in the `data/` folder.
3. Compile the program:
   ```bash
   g++ -std=c++11 src/flight_optimizer.cpp -o flight_optimizer
   ```

## Usage
1. Run the program:
   ```bash
   ./flight_optimizer
   ```
2. Enter source and destination IATA codes (e.g., `LAX` for Los Angeles, `NRT` for Tokyo Narita).
3. View the three fastest routes with estimated distances and additional info.

## Example Output
```
Loaded 7698 airports and 3321 nodes with routes.
Enter source IATA (e.g., LAX): LAX
Enter destination IATA (e.g., NRT): NRT
Top 3 Fastest Routes from LAX to NRT:

1. Fastest Route ($750.00)
LAX -> NRT (5447 miles)
   - Clear skies
   - Priority boarding available

2. Fastest Route (Second) ($900.00)
LAX -> SFO -> NRT (5578 miles)
   - No weather delays
   - Business class upgrade

3. Fastest Route (Third) ($1100.00)
LAX -> HNL -> NRT (6142 miles)
   - Tropical weather advisory
   - 5-star lounge access
```

## Data Sources
- Airports: https://raw.githubusercontent.com/jpatokal/openflights/master/data/airports.dat
- Routes: https://raw.githubusercontent.com/jpatokal/openflights/master/data/routes.dat

## Notes
- The program assumes direct routes and computes distinct paths by excluding used edges.
- Costs are simulated using random weights based on distance.
- Ensure `data/` folder contains the CSV files before running.