#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <set>
#include <limits>
using namespace std;

void loadComponentsFromFile(vector<vector<int>>& adj_list, vector<int>& vertices) {
    fstream file("graph1.txt");
    int rows = 0, cols = 2;

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
void bfs(int v, const vector<vector<int>>& adj_list, vector<bool>& visited, set<int>& component) {
    queue<int> q;
    q.push(v);
    visited[v] = true;
    component.insert(v);
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int w : adj_list[u]) {
            if (!visited[w]) {
                visited[w] = true;
                component.insert(w);
                q.push(w);
            }
        }
    }
}

pair<double, double> floyd_warshall(const vector<vector<int>>& adj_list, const set<int>& component) {
    int n = component.size();
    vector<vector<double>> shortest_paths(n, vector<double>(n, numeric_limits<double>::infinity()));
    vector<int> component_index(component.begin(), component.end()); // indexy vrcholů v komponentě

    // Inicializujeme nejkratší vzdálenosti mezi sousedními vrcholy v komponentě
    for (int u : component) {
        int i = distance(component.begin(), find(component.begin(), component.end(), u));
        for (int v : adj_list[u]) {
            if (component.find(v) != component.end()) {
                int j = distance(component.begin(), find(component.begin(), component.end(), v));
                shortest_paths[i][j] = 1;
                shortest_paths[j][i] = 1;
            }
        }
    }

    // Floydův-Warshallův algoritmus pro nalezení nejkratších cest mezi všemi dvojicemi vrcholů v komponentě
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                shortest_paths[i][j] = min(shortest_paths[i][j], shortest_paths[i][k] + shortest_paths[k][j]);
            }
        }
    }

    // Spočítáme excentricitu (největší vzdálenost) každého vrcholu v komponentě a použijeme ji k výpočtu poloměru a průměru
    double radius = numeric_limits<double>::infinity();
    double diameter = 0;
    for (int i = 0; i < n; i++) {
        double eccentricity = 0;
        for (int j = 0; j < n; j++) {
            eccentricity = max(eccentricity, shortest_paths[i][j]);
        }
        radius = min(radius, eccentricity);
        diameter = max(diameter, eccentricity);
    }

    return make_pair(radius, diameter);
}

int main() {
    // Vektory vrcholu grafu
    vector<int> vertices;
    // Seznam sousednosti
    vector<vector<int>> adj_list;

    loadComponentsFromFile(adj_list, vertices);

    // Hledání největší komponenty
    vector<bool> visited(vertices.size(), false);
    set<int> largest_component;
    vector<set<int>> components;
    for (int v = 0; v < vertices.size(); v++) {
        if (!visited[v]) {
            set<int> component;
            bfs(v, adj_list, visited, component);
            components.push_back(component);
            if (component.size() > largest_component.size()) {
                largest_component = component;
            }
        }
    }

    cout << "Largest component found" << endl;

    int largest_num_vertices = largest_component.size();
    int largest_num_edges = 0;
    int largest_min_degree = largest_num_vertices;
    int largest_max_degree = 0;
    int largest_degree_sum = 0;
    vector<int> degree_histogram(largest_num_vertices + 1, 0); // Histogram rozdělení stupňů vrcholů
    for (int v : largest_component) {
        int degree = adj_list[v].size();
        largest_num_edges += degree;
        largest_degree_sum += degree;
        largest_min_degree = min(largest_min_degree, degree);
        largest_max_degree = max(largest_max_degree, degree);
    }

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
    pair<double, double> metrics = floyd_warshall(adj_list, largest_component);
    cout << "Radius: " << metrics.first << endl;
    cout << "Diameter: " << metrics.second << endl;
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
