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
    size_t a, b, num;
    edge() : a(0), b(0), num(0) {}
    edge(size_t a1, size_t b1, size_t num1) : a(a1), b(b1), num(num1) {}
};

struct network {
    size_t n, m;
    long long k;
    long long total_flow = 0, total_cost = 0;
    const long long inf = 1e18;
    vector<vector<edge>> edges;
    vector<edge> prev;
    vector<long long> f, cost, cap, p, d;
    bool success = true;

    network(size_t n1, size_t m1, size_t k1) : n(n1), m(m1), k(k1) {
        edges.resize(200);
        prev.resize(200);
        f.resize(9000);
        cost.resize(9000);
        cap.resize(9000);
        p.resize(9000);
        d.resize(9000);
    }
};

void initial_dijikstra(network& N) {
    set<pair<long long, size_t>> s;
    s.insert({0, 0});
    for (size_t i = 1; i < N.n; ++i) {
        N.p[i] = N.inf;
        s.insert({N.inf, i});
    }
    while (!s.empty()) {
        size_t cur = s.begin()->second;
        s.erase(s.begin());
        for (edge& e : N.edges[cur]) {
            if (N.cap[e.num] > N.f[e.num] && N.p[e.b] > N.p[cur] + N.cost[e.num]) {
                s.erase({N.p[e.b], e.b});
                N.p[e.b] = N.p[cur] + N.cost[e.num];
                s.insert({N.p[e.b], e.b});
            }
        }
    }
}

void dijikstra_with_johnson_potentials(network& N) {
    set<pair<long long, size_t>> s;
    N.d[0] = 0;
    s.insert({0, 0});
    for (size_t i = 1; i < N.n; ++i) {
        N.d[i] = N.inf;
        s.insert({N.inf, i});
    }
    while (!s.empty()) {
        size_t cur = s.begin()->second;
        s.erase(s.begin());
        for (edge& e : N.edges[cur]) {
            if (N.cap[e.num] > N.f[e.num] && N.d[e.b] > N.d[cur] + N.cost[e.num] + N.p[e.a] - N.p[e.b]) {
                s.erase({N.d[e.b], e.b});
                N.d[e.b] = N.d[cur] + N.cost[e.num] + N.p[e.a] - N.p[e.b];
                s.insert({N.d[e.b], e.b});
                N.prev[e.b] = e;
            }
        }
    }
    for (size_t i = 0; i < N.n; ++i) {
        if (N.p[i] < N.inf)
            N.p[i] += N.d[i];
    }
}

void add_cheapest_path(network& N) {
    vector<edge> path;
    long long min_add = N.inf, v = N.n - 1;
    while (v != 0) {
        edge e = N.prev[v];
        path.push_back(e);
        min_add = min(min_add, N.cap[e.num] - N.f[e.num]);
        v = e.a;
    }
    min_add = min(min_add, N.k - N.total_flow);
    N.total_flow += min_add;
    for (edge e : path) {
        N.f[e.num] += min_add;
        N.total_cost += min_add * N.cost[e.num];
        if (e.num < 4000) {
            N.f[e.num + 4000] -= min_add;
            N.total_cost -= min_add * N.cost[e.num + 4000];
        } else {
            N.f[e.num - 4000] -= min_add;
            N.total_cost -= min_add * N.cost[e.num - 4000];
        }
    }
}

network read() {
    size_t n, m, k;
    cin >> n >> m >> k;
    network N(n, m, k);
    for (size_t i = 0; i < m; ++i) {
        size_t a, b, t = 2 * i;
        cin >> a >> b >> N.cost[t];
        N.cost[t + 4000] = -N.cost[t];
        N.cost[t + 1] = N.cost[t];
        N.cost[t + 4001] = -N.cost[t];
        N.cap[t] = 1;
        N.edges[a - 1].push_back(edge(a - 1, b - 1, t));
        N.cap[t + 4000] = 0;
        N.edges[b - 1].push_back(edge(b - 1, a - 1, t + 4000));
        N.cap[t + 1] = 1;
        N.edges[b - 1].push_back(edge(b - 1, a - 1, t + 1));
        N.cap[t + 4001] = 0;
        N.edges[a - 1].push_back(edge(a - 1, b - 1, t + 4001));
    }
    return N;
}

bool get_path_dfs(int v, vector<size_t>& path, vector<bool>& used, network& N) {
    if (v == N.n - 1) {
        cout << path.size() << " ";
        for (int num : path)
            cout << num + 1 << " ";
        cout << "\n";
        path.clear();
        return true;
    } else {
        for (edge e : N.edges[v]) {
            if (!used[e.num] && N.f[e.num] == 1) {
                used[e.num] = true;
                path.push_back(e.num / 2);
                if (get_path_dfs(e.b, path, used, N))
                    return true;
            }
        }
        return false;
    }
}

void write_ans(network& N) {
    if (N.success) {
        N.total_cost /= 2;
        cout << fixed << setprecision(20) << (long double) N.total_cost / (long double) N.k << "\n";
        vector<size_t> path;
        vector<bool> used(9000);
        for (edge e : N.edges[0]) {
            if (!used[e.num] && N.f[e.num] == 1) {
                used[e.num] = true;
                path.push_back(e.num / 2);
                get_path_dfs(e.b, path, used, N);
            }
        }
    } else {
        cout << -1 << "\n";
    }
}

void solve() {
    network N = read();
    initial_dijikstra(N);
    while (true) {
        if (N.total_flow == N.k)
            break;
        else {
            dijikstra_with_johnson_potentials(N);
            if (N.d[N.n - 1] == N.inf) {
                N.success = false;
                break;
            }
            add_cheapest_path(N);
        }
    }
    write_ans(N);
}

int main() {
    solve();
}
