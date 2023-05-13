#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <algorithm>
#include <climits>
#include <numeric>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
//https://github.com/bshoshany/thread-pool
#include "BS_thread_pool.hpp"
using namespace std;
// mutex pro synchronizaci vláken
std::mutex mtx;

//Nacteni vrcholu grafu a seznamu sousednosti ze souboru
void loadComponentsFromFile(const string& filename, unordered_map<int, vector<int>>& adj_list, unordered_set<int>& vertices) {
    cout << "Loading from file started" << endl;
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "Chyba pri otevirani souboru" << endl;
        exit(1);
    }

    int num1, num2;
    while (file >> num1 >> num2) {
        vertices.insert(num1);
        vertices.insert(num2);
        adj_list[num1].push_back(num2);
        adj_list[num2].push_back(num1);
    }
    cout << "File loaded" << endl;
}

//Prohledani grafu do sirky a oznaceni navstivenych vrcholu
unordered_set<int> bfs_component(int v, const unordered_map<int, vector<int>>& adj_list, unordered_map<int, bool>& visited) {
    unordered_set<int> component;
    queue<int> q;
    q.push(v);
    visited[v] = true;
    while (!q.empty()) {
        int u = q.front();
        q.pop();
        component.insert(u);
        for (int w : adj_list.at(u)) {
            if (!visited[w]) {
                visited[w] = true;
                q.push(w);
            }
        }
    }

    return component;
}

//Ziskani komponent grafu
vector<unordered_set<int>> getComponents(const unordered_set<int>& vertices, const unordered_map<int, vector<int>>& adj_list) {
    cout << "Components find started" << endl;
    unordered_map<int, bool> visited;
    for (int v : vertices) visited[v] = false;
    vector<unordered_set<int>> components;
    for (int v : vertices) {
        if (!visited[v]) {
            components.push_back(bfs_component(v, adj_list, visited));
        }
    }
    cout << "Components found" << endl;
    return components;
}

//Nalezeni nejvetsi komponenty
unordered_set<int> getLargestComponent(const vector<unordered_set<int>>& components) {
    cout << "Largest component find started" << endl;
    unordered_set<int> largest_component;
    for (const auto& component : components)
        if (component.size() > largest_component.size()) {
            largest_component = component;
        }
    cout << "Largest component found" << endl;
    return largest_component;
}

//void calculateGraphStats(const unordered_set<int>& component, const unordered_map<int, vector<int>>& adj_list, double& radius, double& diameter) {
//    cout << "Calculate radius and diameter started" << endl;
//    double min_ecc = numeric_limits<int>::max();
//    double max_ecc = -1;
//
//    BS::thread_pool pool(std::thread::hardware_concurrency() - 1);
//
//    std::mutex mtx;
//
//    for (int v : component) {
//        pool.submit([&, v]() {
//            unordered_map<int, int> dist = bfs_thread(adj_list, v, min_ecc, max_ecc);
//            int max_dist = numeric_limits<int>::min();
//            for (const auto& pair : dist) {
//                if (pair.second != numeric_limits<int>::max()) {
//                    max_dist = max(max_dist, pair.second);
//                }
//            }
//            std::lock_guard<std::mutex> lock(mtx);
//            min_ecc = min(min_ecc, max_dist);
//            max_ecc = max(max_ecc, max_dist);
//            });
//    }
//
//    pool.wait_for_tasks();
//
//    radius = min_ecc;
//    diameter = max_ecc;
//
//    cout << "Calculate radius and diameter ended" << endl;
//}

//void calculateGraphStats(const unordered_set<int>& component, const unordered_map<int, vector<int>>& adj_list, double& radius, double& diameter) {
//    cout << "Calculate radius and diameter started" << endl;
//    int min_ecc = numeric_limits<int>::max();
//    int max_ecc = -1;
//
//    for (int v : component) {
//        unordered_map<int, int> dist = bfs(adj_list, v, numeric_limits<int>::max());
//        int max_dist = numeric_limits<int>::min();
//        for (const auto& pair : dist) {
//            if (pair.second != numeric_limits<int>::max()) {
//                max_dist = max(max_dist, pair.second);
//            }
//        }
//        min_ecc = min(min_ecc, max_dist);
//        max_ecc = max(max_ecc, max_dist);
//    }
//
//    radius = min_ecc;
//    diameter = max_ecc;
//
//    cout << "Calculate radius and diameter ended" << endl;
//}

//Prochazeni pomoci breadth-first search algoritmu k zjisteni vzdalenosti ze startovniho bodu
vector<int> bfs_dist(int start, const unordered_map<int, vector<int>>& adj_list) {
    int n = adj_list.size();
    vector<int> dist(n, numeric_limits<int>::max());
    queue<int> q;
    q.push(start);
    dist[start] = 0;

    while (!q.empty()) {
        int u = q.front();
        q.pop();
        for (int v : adj_list.at(u)) {
            if (dist[v] == numeric_limits<int>::max()) {
                dist[v] = dist[u] + 1;
                q.push(v);
            }
        }
    }

    return dist;
}

atomic<double> radius(numeric_limits<double>::max());
atomic<double> diameter(0.0);

//Vypocet prumeru a polomeru komponenty
void calculateGraphStats(const unordered_set<int>& component, const unordered_map<int, vector<int>>& adj_list) {
    cout << "calculating excentricities started" << endl;
    int num_pairs = 0;
    int i = 0;

    BS::thread_pool pool(thread::hardware_concurrency());

    for (int u : component) {
        pool.submit([&, u]() {
            i++;
            vector<int> dist = bfs_dist(u, adj_list);
            int max_dist = 0;
            for (int v : component) {
                if (u != v && dist[v] != numeric_limits<int>::max()) {
                    max_dist = max(max_dist, dist[v]);
                }
            }

            double old_radius;
            do {
                old_radius = radius.load();
            } while (old_radius > max_dist && !radius.compare_exchange_weak(old_radius, max_dist));

            double old_diameter;
            do {
                old_diameter = diameter.load();
            } while (old_diameter < max_dist && !diameter.compare_exchange_weak(old_diameter, max_dist));

            cout << i << endl;
         });
    }

    pool.wait_for_tasks();  // wait for all tasks to finish

    cout << "calculating excentricities end" << endl;
}

void computeGraphStats(const unordered_set<int>& vertices, const unordered_map<int, vector<int>>& adj_list, const vector<unordered_set<int>>& components) {
    unordered_set<int> largest_component = getLargestComponent(components);

    // Computing stats for the largest component
    int largest_num_vertices = largest_component.size();
    int largest_num_edges = 0;
    int largest_min_degree = INT_MAX;
    int largest_max_degree = INT_MIN;
    int largest_degree_sum = 0;

    unordered_map<int, int> degree_histogram;
    for (int v : largest_component) {
        int degree = adj_list.at(v).size();
        largest_num_edges += degree;
        largest_degree_sum += degree;
        largest_min_degree = min(largest_min_degree, degree);
        largest_max_degree = max(largest_max_degree, degree);

        degree_histogram[degree]++;
    }

    double avg_degree = static_cast<double>(largest_degree_sum) / largest_num_vertices;

    //Vypocet prumeru a polomeru komponenty
    calculateGraphStats(largest_component, adj_list);

    cout << "Graph stats" << endl;
    cout << "Number of Vertices: " << vertices.size() << endl;
    cout << "Number of Edges: " << largest_num_edges / 2 << endl;
    cout << "Number of Components: " << components.size() << endl << endl;

    cout << "Largest Component" << endl;
    cout << "Number of Vertices: " << largest_num_vertices << endl;
    cout << "Number of Edges: " << largest_num_edges / 2 << endl;
    cout << "Min degree: " << largest_min_degree << endl;
    cout << "Max degree: " << largest_max_degree << endl;
    cout << "Avg degree: " << avg_degree << endl;
    cout << "Radius: " << radius.load() << endl;
    cout << "Diameter: " << diameter.load() << endl << endl;

    cout << "Histogram of the peak degree distribution:" << endl;
    for (const auto& degree : degree_histogram) {
        cout << degree.first << ": " << degree.second << endl;
    }
}

int main() {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    // Graph vertices
    unordered_set<int> vertices;
    // Adjacency list
    unordered_map<int, vector<int>> adj_list;

    loadComponentsFromFile("graph1.txt", adj_list, vertices);
    vector<unordered_set<int>> components = getComponents(vertices, adj_list);
    computeGraphStats(vertices, adj_list, components);

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

    std::cout << "Cas behu programu: " << duration.count() << " sekund." << std::endl;

    return 0;
}

