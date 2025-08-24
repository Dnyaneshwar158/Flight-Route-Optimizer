# Flight Route Optimizer

A C++ program to find optimal flight routes between airports using Dijkstra's algorithm.

## Overview
This program finds the most economical, fastest, and scenic flight routes between two airports based on IATA codes. It uses a dataset of global airports and routes, applying Dijkstra's algorithm with weighted edges for distance, cost, and scenic factors.

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
2. Enter source and destination IATA codes (e.g., `JFK` for New York, `SYD` for Sydney).
3. View the top 3 routes: economical, fastest, and scenic, with estimated costs, distances, and additional info.

## Example Output
```
Loaded 7698 airports and 3321 nodes with routes.
Enter source IATA (e.g., JFK): JFK
Enter destination IATA (e.g., SYD): SYD
Top 3 Recommended Routes from JFK to SYD:

1. Most Economical ($1234.56)
JFK -> LAX -> SYD (16035 miles)
   - No weather delays
   - Business class upgrade

2. Fastest Connection ($1500.00)
JFK -> LAX -> SYD (16035 miles)
   - Clear skies
   - Priority boarding available

3. Scenic Route ($1800.00)
JFK -> HNL -> SYD (16253 miles)
   - Tropical weather advisory
   - 5-star lounge access
```

## Data Sources
- Airports: https://raw.githubusercontent.com/jpatokal/openflights/master/data/airports.dat
- Routes: https://raw.githubusercontent.com/jpatokal/openflights/master/data/routes.dat

## Notes
- The program assumes direct routes (no stops).
- Costs and scenic factors are simulated using random weights.
- Ensure `data/` folder contains the CSV files before running.