#include "application.h"

#include <iostream>
#include <limits>
#include <map>
#include <queue>  // priority_queue
#include <set>
#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "dist.h"
#include "graph.h"
#include "json.hpp"


using namespace std;

double INF = numeric_limits<double>::max();

  
void buildGraph(istream& input, graph<long long, double>& G, vector<BuildingInfo>& buildings) {
    using json = nlohmann::json;
    json data;
    input >> data;

    // Parse buildings and add them to the list
    if (data.contains("buildings")) {
        for (const auto& building : data["buildings"]) {
            long long id = building["id"];
            double lat = building["lat"];
            double lon = building["lon"];
            string name = building["name"];
            string abbr = building["abbr"];
            buildings.emplace_back(id, Coordinates(lat, lon), name, abbr);

            // Add the building as a vertex in the graph
            G.addVertex(id);
        }
    }

    // Parse waypoints and add them as vertices
    unordered_map<long long, Coordinates> waypointMap;
    if (data.contains("waypoints")) {
        for (const auto& waypoint : data["waypoints"]) {
            long long id = waypoint["id"];
            double lat = waypoint["lat"];
            double lon = waypoint["lon"];
            G.addVertex(id);
            waypointMap[id] = Coordinates(lat, lon); // Store for distance calculations
        }
    }

    // Parse footways and add edges between consecutive waypoints
    if (data.contains("footways")) {
        for (const auto& footway : data["footways"]) {
            for (size_t i = 0; i < footway.size() - 1; i++) {
                long long from = footway[i];
                long long to = footway[i + 1];

                // Calculate distance between waypoints
                double distance = distBetween2Points(waypointMap[from], waypointMap[to]);

                // Add undirected edges
                G.addEdge(from, to, distance);
                G.addEdge(to, from, distance);
            }
        }
    }

    // Connect each building to nearby waypoints within 0.036 miles
    for (const auto& building : buildings) {
        for (const auto& [waypointId, coords] : waypointMap) {
            double distance = distBetween2Points(building.location, coords);
            if (distance <= 0.036) {
                // Add undirected edges between the building and the waypoint
                G.addEdge(building.id, waypointId, distance);
                G.addEdge(waypointId, building.id, distance);
            }
        }
    }
}


BuildingInfo getBuildingInfo(const vector<BuildingInfo>& buildings,
                             const string& query) { 
  for (const BuildingInfo& building : buildings) {
    if (building.abbr == query) {
      return building;
    } else if (building.name.find(query) != string::npos) {
      return building;
    }
  }
  BuildingInfo fail;
  fail.id = -1;
  return fail;
}

BuildingInfo getClosestBuilding(const vector<BuildingInfo>& buildings,
                                Coordinates c) {
  double minDestDist = INF;
  BuildingInfo ret = buildings.at(0);
  for (const BuildingInfo& building : buildings) {
    double dist = distBetween2Points(building.location, c);
    if (dist < minDestDist) {
      minDestDist = dist;
      ret = building;
    }
  }
  return ret;
}

vector<long long> dijkstra(const graph<long long, double>& G, long long start,
                           long long target, const set<long long>& ignoreNodes) {
    unordered_map<long long, double> distances;
    unordered_map<long long, long long> predecessors;
    priority_queue<pair<double, long long>, vector<pair<double, long long>>, greater<>> pq;

    for (const auto& i : G.getVertices()) {
        distances[i] = INF;
    }

    distances[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        auto [currentDist, currentVertex] = pq.top();
        pq.pop();

        if (currentVertex != start && currentVertex != target && ignoreNodes.count(currentVertex)) {
            continue;
        }

        if (currentVertex == target) {
            break;
        }

        for (const auto& i : G.neighbors(currentVertex)) {
            
            if (i != start && i != target && ignoreNodes.count(i)) {
              continue; 
            }

            double wght;

            if (!G.getWeight(currentVertex, i, wght)) {
              continue; 
            }

            double newDist = currentDist + wght;

            if (newDist < distances[i]) {
                distances[i] = newDist;
                predecessors[i] = currentVertex;
                pq.emplace(newDist, i);
            }
        }
    }

    
    if (distances[target] == INF) {
      return {}; 
    }

    vector<long long> rote;
    for (long long at = target; at != start; at = predecessors[at]) {
      if (predecessors.find(at) == predecessors.end()) {
        return {}; 
      }
      rote.push_back(at);
    }

    rote.push_back(start);
    reverse(rote.begin(), rote.end());
    return rote;
}




double pathLength(const graph<long long, double>& G,
                  const vector<long long>& path) {
  double length = 0.0;
  double weight;
  for (size_t i = 0; i + 1 < path.size(); i++) {
    bool res = G.getWeight(path.at(i), path.at(i + 1), weight);
    if (!res) {
      return -1;
    }
    length += weight;
  }
  return length;
}

void outputPath(const vector<long long>& path) {
  for (size_t i = 0; i < path.size(); i++) {
    cout << path.at(i);
    if (i != path.size() - 1) {
      cout << "->";
    }
  }
  cout << endl;
}

void application(const vector<BuildingInfo>& buildings,
                 const graph<long long, double>& G) {
  string person1Building, person2Building;

  set<long long> buildingNodes;
  for (const auto& building : buildings) {
    buildingNodes.insert(building.id);
  }

  cout << endl;
  cout << "Enter person 1's building (partial name or abbreviation), or #> ";
  getline(cin, person1Building);

  while (person1Building != "#") {
    cout << "Enter person 2's building (partial name or abbreviation)> ";
    getline(cin, person2Building);

    // Look up buildings by query
    BuildingInfo p1 = getBuildingInfo(buildings, person1Building);
    BuildingInfo p2 = getBuildingInfo(buildings, person2Building);
    Coordinates P1Coords, P2Coords;
    string P1Name, P2Name;

    if (p1.id == -1) {
      cout << "Person 1's building not found" << endl;
    } else if (p2.id == -1) {
      cout << "Person 2's building not found" << endl;
    } else {
      cout << endl;
      cout << "Person 1's point:" << endl;
      cout << " " << p1.name << endl;
      cout << " " << p1.id << endl;
      cout << " (" << p1.location.lat << ", " << p1.location.lon << ")" << endl;
      cout << "Person 2's point:" << endl;
      cout << " " << p2.name << endl;
      cout << " " << p2.id << endl;
      cout << " (" << p2.location.lon << ", " << p2.location.lon << ")" << endl;

      Coordinates centerCoords = centerBetween2Points(p1.location, p2.location);
      BuildingInfo dest = getClosestBuilding(buildings, centerCoords);

      cout << "Destination Building:" << endl;
      cout << " " << dest.name << endl;
      cout << " " << dest.id << endl;
      cout << " (" << dest.location.lat << ", " << dest.location.lon << ")"
           << endl;

      vector<long long> P1Path = dijkstra(G, p1.id, dest.id, buildingNodes);
      vector<long long> P2Path = dijkstra(G, p2.id, dest.id, buildingNodes);

      // This should NEVER happen with how the graph is built
      if (P1Path.empty() || P2Path.empty()) {
        cout << endl;
        cout << "At least one person was unable to reach the destination "
                "building. Is an edge missing?"
             << endl;
        cout << endl;
      } else {
        cout << endl;
        cout << "Person 1's distance to dest: " << pathLength(G, P1Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P1Path);
        cout << endl;
        cout << "Person 2's distance to dest: " << pathLength(G, P2Path);
        cout << " miles" << endl;
        cout << "Path: ";
        outputPath(P2Path);
      }
    }

    //
    // another navigation?
    //
    cout << endl;
    cout << "Enter person 1's building (partial name or abbreviation), or #> ";
    getline(cin, person1Building);
  }
}

