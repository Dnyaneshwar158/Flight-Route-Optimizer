#include <iostream>
#include <unordered_map>
#include <vector>
#include <queue>
#include <string>
#include <limits>
#include <fstream>
#include <sstream>
#include <cmath>
#include <random>
#include <iomanip>

// Constants
#define PI 3.14159265358979323846
#define EARTH_RADIUS 6371.0  // km

// Structs
struct Airport {
    std::string iata;
    double lat;
    double lon;
    Airport(const std::string& iata_, double lat_, double lon_) : iata(iata_), lat(lat_), lon(lon_) {}
};

struct Edge {
    std::string dest;
    double distance;
    double cost;
    double scenic;
    Edge(const std::string& dest_, double dist_, double cost_, double scenic_)
        : dest(dest_), distance(dist_), cost(cost_), scenic(scenic_) {}
};

// Graph type
typedef std::unordered_map<std::string, std::vector<Edge> > Graph;

// Dijkstra result
struct PathResult {
    std::vector<std::string> path;
    double total;
    double miles;
    std::string weather;
    std::string extra;
    PathResult() : total(0.0), miles(0.0) {}
};

// Haversine distance
double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * PI / 180.0;
    double dlon = (lon2 - lon1) * PI / 180.0;
    double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
               std::cos(lat1 * PI / 180.0) * std::cos(lat2 * PI / 180.0) *
               std::sin(dlon / 2) * std::sin(dlon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return EARTH_RADIUS * c;
}

// Simple CSV parser
std::vector<std::string> parse_csv_line(const std::string& line) {
    std::vector<std::string> fields;
    std::string field;
    bool in_quotes = false;
    for (char c : line) {
        if (c == '"') in_quotes = !in_quotes;
        else if (c == ',' && !in_quotes) {
            fields.push_back(field == "\\N" ? "" : field);
            field.clear();
        } else {
            field += c;
        }
    }
    if (!field.empty()) fields.push_back(field == "\\N" ? "" : field);
    return fields;
}

// Load data
bool load_data(std::unordered_map<std::string, Airport>& airports, Graph& graph) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(0.5, 1.5);  // Cost factor
    std::uniform_real_distribution<double> scenic_dis(0.0, 1.0);  // Scenic factor

    // Load airports
    std::ifstream af("data/global_airports.csv");
    if (!af.is_open()) {
        std::cerr << "Error: Cannot open data/global_airports.csv" << std::endl;
        return false;
    }
    std::string line;
    while (std::getline(af, line)) {
        std::vector<std::string> fields = parse_csv_line(line);
        if (fields.size() < 14) continue;
        std::string iata = fields[4];
        if (iata.empty() || iata == "\\N") continue;
        double lat, lon;
        try {
            lat = std::stod(fields[6]);
            lon = std::stod(fields[7]);
        } catch (...) { continue; }
        airports.emplace(iata, Airport(iata, lat, lon));
    }
    af.close();

    // Load routes
    std::ifstream rf("data/global_routes.csv");
    if (!rf.is_open()) {
        std::cerr << "Error: Cannot open data/global_routes.csv" << std::endl;
        return false;
    }
    while (std::getline(rf, line)) {
        std::vector<std::string> fields = parse_csv_line(line);
        if (fields.size() < 9) continue;
        std::string src = fields[2], dest = fields[4];
        std::string stops_str = fields[7];
        if (src.empty() || dest.empty() || airports.count(src) == 0 || airports.count(dest) == 0) continue;
        int stops;
        try { stops = std::stoi(stops_str); } catch (...) { stops = 0; }
        if (stops != 0) continue;  // Only direct routes
        double dist = haversine(airports.at(src).lat, airports.at(src).lon, airports.at(dest).lat, airports.at(dest).lon);
        double c_factor = dis(gen);
        double cost = dist * 0.1 * c_factor;
        double scenic = scenic_dis(gen);
        graph[src].push_back(Edge(dest, dist, cost, scenic));
        graph[dest].push_back(Edge(src, dist, cost, scenic));  // Undirected
    }
    rf.close();
    return true;
}

// Dijkstra
PathResult dijkstra(const Graph& graph, const std::string& start, const std::string& end, int mode) {
    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev;
    typedef std::pair<double, std::string> PQPair;
    std::priority_queue<PQPair, std::vector<PQPair>, std::greater<PQPair> > pq;

    for (const std::pair<std::string, std::vector<Edge> >& node : graph) {
        dist[node.first] = std::numeric_limits<double>::infinity();
    }
    dist[start] = 0.0;
    pq.push(std::make_pair(0.0, start));

    while (!pq.empty()) {
        double curr_dist = pq.top().first;
        std::string curr = pq.top().second;
        pq.pop();
        if (curr_dist > dist[curr]) continue;
        if (curr == end) break;

        const std::vector<Edge>& edges = graph.at(curr);
        for (std::vector<Edge>::const_iterator it = edges.begin(); it != edges.end(); ++it) {
            const Edge& edge = *it;
            double weight;
            if (mode == 0) weight = edge.distance;                     // Fastest
            else if (mode == 1) weight = edge.cost;                    // Economical
            else weight = edge.distance * (1.0 - edge.scenic * 0.5);   // Scenic
            double new_dist = curr_dist + weight;
            if (new_dist < dist[edge.dest]) {
                dist[edge.dest] = new_dist;
                prev[edge.dest] = curr;
                pq.push(std::make_pair(new_dist, edge.dest));
            }
        }
    }

    // Reconstruct
    PathResult res;
    res.total = dist[end];
    if (res.total == std::numeric_limits<double>::infinity()) return res;
    double actual_miles = 0.0;
    std::string at = end;
    while (!at.empty()) {
        res.path.insert(res.path.begin(), at);
        if (!prev[at].empty()) {
            std::string prev_node = prev[at];
            const std::vector<Edge>& prev_edges = graph.at(prev_node);
            for (std::vector<Edge>::const_iterator it = prev_edges.begin(); it != prev_edges.end(); ++it) {
                if (it->dest == at) {
                    actual_miles += it->distance;
                    break;
                }
            }
        }
        at = prev[at];
    }
    res.miles = actual_miles;

    // Mock weather/extra
    std::vector<std::string> weathers = {"No weather delays", "Tropical weather advisory", "Clear skies"};
    std::vector<std::string> extras = {"Priority boarding available", "Business class upgrade", "5-star lounge access", "2 stops maximum"};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> w_dis(0, weathers.size() - 1);
    std::uniform_int_distribution<int> e_dis(0, extras.size() - 1);
    res.weather = weathers[w_dis(gen)];
    res.extra = extras[e_dis(gen)];

    return res;
}

// Main CLI
int main() {
    std::unordered_map<std::string, Airport> airports;
    Graph graph;
    if (!load_data(airports, graph)) {
        std::cerr << "Failed to load data. Check data/ folder." << std::endl;
        return 1;
    }
    std::cout << "Loaded " << airports.size() << " airports and " << graph.size() << " nodes with routes." << std::endl;

    std::string start, end;
    std::cout << "Enter source IATA (e.g., JFK): ";
    std::cin >> start;
    std::cout << "Enter destination IATA (e.g., SYD): ";
    std::cin >> end;

    if (graph.count(start) == 0 || graph.count(end) == 0) {
        std::cout << "Invalid airports." << std::endl;
        return 1;
    }

    // Compute routes
    PathResult econ = dijkstra(graph, start, end, 1);  // Cost
    PathResult fast = dijkstra(graph, start, end, 0);  // Distance
    PathResult scenic = dijkstra(graph, start, end, 2);  // Scenic

    // Output
    std::cout << "Top 3 Recommended Routes from " << start << " to " << end << ":" << std::endl << std::endl;
    std::cout << "1. Most Economical ($" << std::fixed << std::setprecision(2) << econ.total << ")" << std::endl;
    for (size_t i = 0; i < econ.path.size(); ++i) {
        std::cout << econ.path[i];
        if (i < econ.path.size() - 1) std::cout << " -> ";
    }
    std::cout << " (" << static_cast<int>(econ.miles) << " miles)" << std::endl;
    std::cout << "   - " << econ.weather << std::endl;
    std::cout << "   - " << econ.extra << std::endl << std::endl;

    std::cout << "2. Fastest Connection ($" << std::fixed << std::setprecision(2) << fast.total * 0.1 << ")" << std::endl;
    for (size_t i = 0; i < fast.path.size(); ++i) {
        std::cout << fast.path[i];
        if (i < fast.path.size() - 1) std::cout << " -> ";
    }
    std::cout << " (" << static_cast<int>(fast.miles) << " miles)" << std::endl;
    std::cout << "   - " << fast.weather << std::endl;
    std::cout << "   - " << fast.extra << std::endl << std::endl;

    std::cout << "3. Scenic Route ($" << std::fixed << std::setprecision(2) << scenic.total * 0.1 << ")" << std::endl;
    for (size_t i = 0; i < scenic.path.size(); ++i) {
        std::cout << scenic.path[i];
        if (i < scenic.path.size() - 1) std::cout << " -> ";
    }
    std::cout << " (" << static_cast<int>(scenic.miles) << " miles)" << std::endl;
    std::cout << "   - " << scenic.weather << std::endl;
    std::cout << "   - " << scenic.extra << std::endl;

    return 0;
}