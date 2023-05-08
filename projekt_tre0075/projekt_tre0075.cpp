#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <set>
#include <limits>
using namespace std;

void loadComponentsFromFile(const string filename, vector<vector<int>>& adj_list, vector<int>& vertices) {
    cout << "Loading from file started" << endl;
    fstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Chyba pri otevirani souboru" << std::endl;
        exit(1);
    }

    string line;
    int num1, num2;
    while (getline(file, line)) {
        if (sscanf_s(line.c_str(), "%d %d", &num1, &num2) == 2) {
            // Přidání vrcholů do seznamu vrcholů grafu
            if (find(vertices.begin(), vertices.end(), num1) == vertices.end()) {
                vertices.push_back(num1);
            }
            if (find(vertices.begin(), vertices.end(), num2) == vertices.end()) {
                vertices.push_back(num2);
            }
            // Přidání hran do seznamu sousednosti
            int index1 = distance(vertices.begin(), find(vertices.begin(), vertices.end(), num1));
            int index2 = distance(vertices.begin(), find(vertices.begin(), vertices.end(), num2));
            while (adj_list.size() < max(index1, index2) + 1) {
                adj_list.push_back(vector<int>());
            }
            adj_list[index1].push_back(index2);
            adj_list[index2].push_back(index1);
        }
    }
    cout << "File loaded" << endl;
    file.close();
}

// Funkce pro prohledání grafu do šířky a označení navštívených vrcholů
void bfs(int v, const vector<vector<int>>& adj_list, vector<bool>& visited, set<int>& component, vector<double>& max_distance) {
    queue<int> q;
    q.push(v);
    visited[v] = true;
    component.insert(v);
    max_distance[v] = 0;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int w : adj_list[u]) {
            if (!visited[w]) {
                visited[w] = true;
                component.insert(w);
                max_distance[w] = max_distance[u] + 1;
                q.push(w);
            }
        }
    }
}

vector<set<int>> getComponents(vector<int>& vertices, const vector<vector<int>>& adj_list) {
    cout << "Components find started" << endl;
    int n = adj_list.size();
    vector<double> dist(n, numeric_limits<double>::infinity());
    vector<bool> visited(vertices.size(), false);
    vector<set<int>> components;
    for (int v = 0; v < vertices.size(); v++) {
        if (!visited[v]) {
            set<int> component;
            bfs(v, adj_list, visited, component, dist);
            components.push_back(component);
        }
    }
    cout << "Components found" << endl;
    return components;
}

set<int> getLargestComponent(const vector<set<int>>& components) {
    cout << "Largest component find started" << endl;
    set<int> largest_component;
    for (const auto& component : components)
        if (component.size() > largest_component.size()) {
            largest_component = component;
        }
    cout << "Largest component found" << endl;
    return largest_component;
}

vector<int> bfs_dist(int start, const vector<vector<int>>& adj_list) {
    int n = adj_list.size();
    vector<int> dist(n, numeric_limits<int>::max());
    queue<int> q;
    q.push(start);
    dist[start] = 0;

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int v : adj_list[u]) {
            if (dist[v] == numeric_limits<int>::max()) {
                dist[v] = dist[u] + 1;
                q.push(v);
            }
        }
    }

    return dist;
}

void calculateGraphStats(const set<int>& component, const vector<vector<int>>& adj_list, double& radius, double& diameter) {
    double sum = 0.0;
    int num_pairs = 0;
    radius = numeric_limits<double>::max();
    diameter = numeric_limits<double>::min();

    for (int u : component) {
        vector<int> dist = bfs_dist(u, adj_list);
        int max_dist = 0;
        for (int v : component) {
            if (u != v && dist[v] != numeric_limits<int>::max()) {
                max_dist = max(max_dist, dist[v]);
                sum += dist[v];
                num_pairs++;
            }
        }
        radius = min(radius, (double)max_dist);
        diameter = max(diameter, (double)max_dist);
    }

    double avg_distance = sum / num_pairs;
    cout << "Graph stats for largest component" << endl;
    cout << "Radius: " << radius << endl;
    cout << "Diameter: " << diameter << endl;
    cout << "Average distance: " << avg_distance << endl;
}



int main() {
    // Vektory vrcholu grafu
    vector<int> vertices;
    // Seznam sousednosti
    vector<vector<int>> adj_list;

    loadComponentsFromFile("test.txt", adj_list, vertices);
    vector<set<int>> components = getComponents(vertices, adj_list);
    set<int> largest_component = getLargestComponent(components);
    int largest_num_vertices = largest_component.size();
    int largest_num_edges = 0;
    int largest_min_degree = largest_num_vertices;
    int largest_max_degree = 0;
    int largest_degree_sum = 0;
    for (int v : largest_component) {
        int degree = adj_list[v].size();
        largest_num_edges += degree;
        largest_degree_sum += degree;
        largest_min_degree = min(largest_min_degree, degree);
        largest_max_degree = max(largest_max_degree, degree);
    }

    vector<int> degree_histogram(largest_num_vertices + 1, 0); // Histogram rozdělení stupňů vrcholů
    int num_edges_all = 0;
    set<int> all_vertices;
    for (const auto& component : components) {
        int num_vertices = component.size();
        int num_edges = 0;
        for (int v : component) {
            num_edges += adj_list[v].size();
            all_vertices.insert(v);
            int degree = adj_list[v].size();
            degree_histogram[degree]++;
        }
        num_edges_all += num_edges / 2;
    }

    double avg_degree = (double)largest_num_edges / largest_num_vertices;

    vector<vector<pair<int, double>>> adj_list_double(adj_list.size());
    for (int i = 0; i < adj_list.size(); i++) {
        for (int j = 0; j < adj_list[i].size(); j++) {
            int v = adj_list[i][j];
            double weight = 1.0; // Váha hrany
            adj_list_double[i].push_back({ v, weight });
        }
    }

    double radius, diameter;
    calculateGraphStats(largest_component, adj_list, radius, diameter);
    cout << "Graph stats" << endl;
    cout << "Number of Vertices: " << all_vertices.size() << endl;
    cout << "Number of Edges: " << num_edges_all << endl;
    cout << "Number of Components: " << components.size() << endl;
    cout << "Largest Component" << endl;
    cout << "Number of Vertices: " << largest_num_vertices << endl;
    cout << "Number of Edges: " << largest_num_edges / 2 << endl;
    cout << "Min degree: " << largest_min_degree << endl;
    cout << "Max degree: " << largest_max_degree << endl;
    cout << "Avg degree: " << avg_degree << endl;
    cout << "Histogram rozdeleni stupnu vrcholu:" << endl;
    for (int i = 0; i < degree_histogram.size(); i++) {
        if (degree_histogram[i] > 0) {
            cout << i << ": " << degree_histogram[i] << endl;
        }
    }

    system("pause");
    return 0;
}
