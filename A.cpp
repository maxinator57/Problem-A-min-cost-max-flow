#include <iostream>
#include <iomanip>
#include <vector>
#include <set>

using std::vector;
using std::set;
using std::pair;
using std::min;
using std::cin;
using std::cout;
using std::setprecision;
using std::fixed;

struct edge {
    int a = -1, b = -1, num = -1;
    edge() : a(-1), b(-1), num(-1) {}
    edge(int a1, int b1, int num1) {
        a = a1;
        b = b1;
        num = num1;
    }
};

int n, m, k;
long long total_flow = 0, total_cost = 0;
const int maxn = 200, maxm = 9000;
const long long inf = 1e18;
vector<edge> edges[maxn];
edge prev[maxn];
long long f[maxm], cost[maxm], cap[maxm], p[maxn], d[maxn];
bool success = true;

void initial_Dijikstra() {
    set<pair<long long, int>> s;
    s.insert({0, 0});
    for (int i = 1; i < n; ++i) {
        p[i] = inf;
        s.insert({inf, i});
    }
    while (!s.empty()) {
        int cur = s.begin()->second;
        s.erase(s.begin());
        for (edge& e : edges[cur]) {
            if (cap[e.num] > f[e.num] && p[e.b] > p[cur] + cost[e.num]) {
                s.erase({p[e.b], e.b});
                p[e.b] = p[cur] + cost[e.num];
                s.insert({p[e.b], e.b});
            }
        }
    }
}

void Dijikstra_with_Johnson_potentials() {
    set<pair<long long, int>> s;
    d[0] = 0;
    s.insert({0, 0});
    for (int i = 1; i < n; ++i) {
        d[i] = inf;
        s.insert({inf, i});
    }
    while (!s.empty()) {
        int cur = s.begin()->second;
        s.erase(s.begin());
        for (edge& e : edges[cur]) {
            if (cap[e.num] > f[e.num] && d[e.b] > d[cur] + cost[e.num] + p[e.a] - p[e.b]) {
                s.erase({d[e.b], e.b});
                d[e.b] = d[cur] + cost[e.num] + p[e.a] - p[e.b];
                s.insert({d[e.b], e.b});
                prev[e.b] = e;
            }
        }
    }
    for (int i = 0; i < n; ++i) {
        if (p[i] < inf)
            p[i] += d[i];
    }
}

void add_cheapest_path() {
    vector<edge> path;
    long long min_add = inf, v = n - 1;
    while (v != 0) {
        edge e = prev[v];
        path.push_back(e);
        min_add = min(min_add, cap[e.num] - f[e.num]);
        v = e.a;
    }
    min_add = min(min_add, k - total_flow);
    total_flow += min_add;
    for (edge e : path) {
        f[e.num] += min_add;
        total_cost += min_add * cost[e.num];
        if (e.num < 4000) {
            f[e.num + 4000] -= min_add;
            total_cost -= min_add * cost[e.num + 4000];
        } else {
            f[e.num - 4000] -= min_add;
            total_cost -= min_add * cost[e.num - 4000];
        }
    }
}

void read() {
    cin >> n >> m >> k;
    for (int i = 0; i < m; ++i) {
        int a, b, t = 2 * i;
        cin >> a >> b >> cost[t];
        cost[t + 4000] = -cost[t];
        cost[t + 1] = cost[t];
        cost[t + 4001] = -cost[t];
        cap[t] = 1;
        edges[a - 1].push_back(edge(a - 1, b - 1, t));
        cap[t + 4000] = 0;
        edges[b - 1].push_back(edge(b - 1, a - 1, t + 4000));
        cap[t + 1] = 1;
        edges[b - 1].push_back(edge(b - 1, a - 1, t + 1));
        cap[t + 4001] = 0;
        edges[a - 1].push_back(edge(a - 1, b - 1, t + 4001));
    }
}

bool get_path_dfs(int v, vector<int>& path, vector<bool>& used) {
    if (v == n - 1) {
        cout << path.size() << " ";
        for (int num : path)
            cout << num + 1 << " ";
        cout << "\n";
        path.clear();
        return true;
    } else {
        for (edge e : edges[v]) {
            if (!used[e.num] && f[e.num] == 1) {
                used[e.num] = true;
                path.push_back(e.num / 2);
                if (get_path_dfs(e.b, path, used))
                    return true;
            }
        }
        return false;
    }
}

void solve() {
    read();
    initial_Dijikstra();
    while (true) {
        if (total_flow == k)
            break;
        else {
            Dijikstra_with_Johnson_potentials();
            if (d[n - 1] == inf) {
                success = false;
                break;
            }
            add_cheapest_path();
        }
    }
    if (success) {
        total_cost /= 2;
        cout << fixed << setprecision(20) << (long double) total_cost / (long double) k << "\n";
        vector<int> path;
        vector<bool> used(maxm);
        for (edge e : edges[0]) {
            if (!used[e.num] && f[e.num] == 1) {
                used[e.num] = true;
                path.push_back(e.num / 2);
                get_path_dfs(e.b, path, used);
            }
        }
    } else {
        cout << -1 << "\n";
    }
}

int main() {
    solve();
}
