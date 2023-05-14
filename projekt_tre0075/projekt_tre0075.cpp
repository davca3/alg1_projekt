/**
 * @file    main.cpp
 * @author  David Tretina (TRE0075) <david.tretina.st@vsb.cz>
 * @version 1.1
 *
 * @brief   A program for analyzing and calculating statistics of a graph.
 */

#include <iostream>
#include <string>
#include <fstream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <chrono>
#include <thread>
#include <atomic>
#include "BS_thread_pool.hpp" //https://github.com/bshoshany/thread-pool
using namespace std;
atomic<double> radius(numeric_limits<double>::max());
atomic<double> diameter(0.0);

/**
 * @brief Loads graph vertices and adjacency list from a file.
 *
 * @param filename The name of the file to load.
 * @param adj_list The adjacency list to fill.
 * @param vertices The set of vertices to fill.
 */
void loadComponentsFromFile(const string& filename, unordered_map<int, vector<int>>& adj_list, unordered_set<int>& vertices) {
    //cout << "Loading from file started" << endl;
    ifstream file(filename);

    if (!file.is_open()) {
        cerr << "File load failed" << endl;
        exit(1);
    }

    int num1, num2;
    while (file >> num1 >> num2) {
        vertices.insert(num1);
        vertices.insert(num2);
        adj_list[num1].push_back(num2);
        adj_list[num2].push_back(num1);
    }
    //cout << "File loaded" << endl;
}

/**
 * @brief Performs a breadth-first search on the graph and marks visited vertices.
 *
 * @param v The starting vertex.
 * @param adj_list The adjacency list of the graph.
 * @param visited A map to track visited vertices.
 * @return The set of vertices in the component.
 */
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

/**
 * @brief Finds all components in the graph.
 *
 * @param vertices The set of vertices in the graph.
 * @param adj_list The adjacency list of the graph.
 * @return A vector containing sets of vertices for each component.
 */
vector<unordered_set<int>> getComponents(const unordered_set<int>& vertices, const unordered_map<int, vector<int>>& adj_list) {
    //cout << "Components find started" << endl;
    unordered_map<int, bool> visited;
    for (int v : vertices) visited[v] = false;
    vector<unordered_set<int>> components;
    for (int v : vertices) {
        if (!visited[v]) {
            components.push_back(bfs_component(v, adj_list, visited));
        }
    }
    //cout << "Components found" << endl;
    return components;
}

/**
 * @brief Finds the largest component in the graph.
 *
 * @param components A vector containing sets of vertices for each component.
 * @return The largest component as a set of vertices.
 */
unordered_set<int> getLargestComponent(const vector<unordered_set<int>>& components) {
    //cout << "Largest component find started" << endl;
    unordered_set<int> largest_component;
    for (const auto& component : components)
        if (component.size() > largest_component.size()) {
            largest_component = component;
        }
    //cout << "Largest component found" << endl;
    return largest_component;
}

/**
 * @brief Performs a breadth-first search to calculate the distance from a starting vertex to all other vertices.
 *
 * @param start The starting vertex.
 * @param adj_list The adjacency list of the graph.
 * @return A vector containing the distances from the starting vertex to all other vertices.
 */
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

/**
 * @brief Calculates the radius and diameter of a graph component.
 *
 * @param component The set of vertices in the component.
 * @param adj_list The adjacency list of the graph.
 */
void calculateGraphStats(const unordered_set<int>& component, const unordered_map<int, vector<int>>& adj_list) {
    //cout << "calculating excentricities started" << endl;
    int num_pairs = 0;

    BS::thread_pool pool(thread::hardware_concurrency());

    for (int u : component) {
        pool.submit([&, u]() {
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
                    });
    }

    pool.wait_for_tasks();  // wait for all tasks to finish

    //cout << "calculating excentricities end" << endl;
}

/**
 * @brief Computes various statistics of the graph.
 *
 * @param vertices The set of vertices in the graph.
 * @param adj_list The adjacency list of the graph.
 * @param components A vector containing sets of vertices for each component.
*/
void computeGraphStats(const unordered_set<int>& vertices, const unordered_map<int, vector<int>>& adj_list, const vector<unordered_set<int>>& components) {
    unordered_set<int> largest_component = getLargestComponent(components);
    int largest_num_vertices = largest_component.size();
    int largest_num_edges = 0;
    int largest_min_degree = INT_MAX;
    int largest_max_degree = INT_MIN;
    int largest_degree_sum = 0;
    vector<int> degree_histogram(largest_num_vertices + 1, 0);
    for (int v : largest_component) {
        int degree = adj_list.at(v).size();
        largest_num_edges += degree;
        largest_degree_sum += degree;
        largest_min_degree = min(largest_min_degree, degree);
        largest_max_degree = max(largest_max_degree, degree);
        degree_histogram[degree]++;
    }

    double avg_degree = static_cast<double>(largest_degree_sum) / largest_num_vertices;
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
    for (int i = 0; i < degree_histogram.size(); i++) {
        if (degree_histogram[i] > 0) {
            cout << i << ": " << degree_histogram[i] << endl;
        }
    }
}

int main() {
    chrono::high_resolution_clock::time_point start = chrono::high_resolution_clock::now();

    unordered_set<int> vertices;
    unordered_map<int, vector<int>> adj_list;

    loadComponentsFromFile("graph1.txt", adj_list, vertices);
    vector<unordered_set<int>> components = getComponents(vertices, adj_list);
    computeGraphStats(vertices, adj_list, components);

    chrono::high_resolution_clock::time_point end = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = chrono::duration_cast<chrono::duration<double>>(end - start);

    cout << "Program ran: " << duration.count() << " seconds" << endl;

    return 0;
}