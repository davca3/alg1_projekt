#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <queue>
#include <set>
using namespace std;

// Funkce pro prohledání grafu do hloubky a označení navštívených vrcholů
void dfs(int v, const vector<vector<int>>& adj_list, vector<bool>& visited, set<int>& component) {
    visited[v] = true;
    component.insert(v);
    for (int u : adj_list[v]) {
        if (!visited[u]) {
            dfs(u, adj_list, visited, component);
        }
    }
}

int main()
{
    string line;
    fstream readFile("test.txt");
    int rows = 0, cols = 2;

    // Vytvoření vektoru dvojic
    vector<pair<int, int>> pairs;

    while (getline(readFile, line)) {
        int num1, num2;
        if (sscanf_s(line.c_str(), "%d %d", &num1, &num2) == 2) {
            // Přidání dvojice do vektoru
            pairs.push_back(make_pair(num1, num2));
        }
    }

    readFile.close();

    // Vytvoření seznamu sousednosti pro každý vrchol grafu
    int max_vertex = 0;
    for (const auto& p : pairs) {
        max_vertex = max(max_vertex, max(p.first, p.second));
    }
    vector<vector<int>> adj_list(max_vertex + 1);
    for (const auto& p : pairs) {
        adj_list[p.first].push_back(p.second);
        adj_list[p.second].push_back(p.first);
    }

    // Hledání největší komponenty
    vector<bool> visited(max_vertex + 1, false);
    set<int> largest_component;
    for (int v = 1; v <= max_vertex; v++) {
        if (!visited[v]) {
            set<int> component;
            dfs(v, adj_list, visited, component);
            if (component.size() > largest_component.size()) {
                largest_component = component;
            }
        }
    }

    // Výpis největší komponenty
    cout << "Nejvetsi komponenta obsahuje " << largest_component.size() << " vrcholu:" << endl;
    for (int v : largest_component) {
        cout << v << endl;
    }
}
