// Situacion problema - Ruta peatonal con menor exposicion al sol en el campus
// Reporte de avance

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <queue>
#include <random>
#include <string>
#include <vector>

using namespace std;

const double INF = numeric_limits<double>::infinity();
const double EPS = 1e-9;
const double V = 1.25;  // velocidad de caminata en m/s

struct Tramo {
    int destino;
    int largo;              // metros, redondeado hacia arriba
    vector<double> sombra;  // fraccion a la sombra en cada franja
};

struct Mapa {
    vector<string> nombres;
    vector<vector<Tramo>> ady;
    vector<int> inicios;  // segundo del dia en que empieza cada franja

    int agregarPunto(const string& nombre) {
        nombres.push_back(nombre);
        ady.push_back({});
        return (int)nombres.size() - 1;
    }

    // O(1)
    void agregarTramo(int u, int w, int largo, const vector<double>& sombra) {
        ady[u].push_back({w, largo, sombra});
        ady[w].push_back({u, largo, sombra});
    }
};

struct Resultado {
    bool hayRuta = false;
    double sol = INF;
    int largo = 0;
    vector<int> ruta;
    bool usoRespaldo = false;
};

// O(E log V)
// Distancia minima de cada punto hacia d, solo por longitud
vector<double> dijkstra(const Mapa& m, int d) {
    int n = (int)m.ady.size();
    vector<double> dist(n, INF);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    dist[d] = 0;
    pq.push({0, d});
    while (!pq.empty()) {
        auto [du, u] = pq.top();
        pq.pop();
        if (du > dist[u]) continue;
        for (const Tramo& t : m.ady[u]) {
            if (du + t.largo < dist[t.destino]) {
                dist[t.destino] = du + t.largo;
                pq.push({dist[t.destino], t.destino});
            }
        }
    }
    return dist;
}

// O(|H|)
// Metros al sol de un tramo que empieza despues de caminar x metros
// Parte el tramo en los pedazos que caen en cada franja
double solTramo(const Mapa& m, const Tramo& t, double x, int t0) {
    int h = (int)m.inicios.size();
    double total = 0;
    for (int k = 0; k < h; k++) {
        double a = (k == 0) ? -INF : V * (m.inicios[k] - t0);
        double b = (k + 1 < h) ? V * (m.inicios[k + 1] - t0) : INF;
        double lo = max(a, x), hi = min(b, x + t.largo);
        if (hi > lo) total += (hi - lo) * (1 - t.sombra[k]);
    }
    return total;
}

// O(1)
bool mejor(double s1, int l1, double s2, int l2) {
    return s1 < s2 - EPS || (fabs(s1 - s2) <= EPS && l1 < l2);
}

// O(V)
bool esSimple(const vector<int>& ruta, int n) {
    vector<bool> visto(n, false);
    for (int u : ruta) {
        if (visto[u]) return false;
        visto[u] = true;
    }
    return true;
}

// Exponencial
// Busqueda exhaustiva con poda por Lmax y por el mejor sol encontrado
// simples = true solo acepta rutas sin puntos repetidos
Resultado fuerzaBruta(const Mapa& m, int o, int d, double lmax, int t0, bool simples) {
    vector<double> r = dijkstra(m, d);
    Resultado res;
    vector<int> ruta = {o};
    vector<bool> enRuta(m.ady.size(), false);
    enRuta[o] = true;

    function<void(int, int, double)> dfs = [&](int u, int L, double S) {
        if (L + r[u] > lmax + EPS) return;
        if (S > res.sol + EPS) return;
        if (u == d && mejor(S, L, res.sol, res.largo)) {
            res.hayRuta = true;
            res.sol = S;
            res.largo = L;
            res.ruta = ruta;
        }
        for (const Tramo& t : m.ady[u]) {
            if (simples && enRuta[t.destino]) continue;
            ruta.push_back(t.destino);
            if (simples) enRuta[t.destino] = true;
            dfs(t.destino, L + t.largo, S + solTramo(m, t, L, t0));
            if (simples) enRuta[t.destino] = false;
            ruta.pop_back();
        }
    };
    dfs(o, 0, 0.0);
    return res;
}

// O(Lmax * E)
// Programacion dinamica sobre el estado (metros caminados, punto)
// F[x][u] = menor sol para llegar a u despues de caminar exactamente x metros
Resultado programacionDinamica(const Mapa& m, int o, int d, double lmax, int t0,
                               const vector<double>& r) {
    int n = (int)m.ady.size();
    int X = (int)floor(lmax + EPS);
    vector<vector<double>> F(X + 1, vector<double>(n, INF));
    vector<vector<pair<int, int>>> P(X + 1, vector<pair<int, int>>(n, {-1, -1}));
    F[0][o] = 0;

    for (int x = 0; x <= X; x++) {
        for (int u = 0; u < n; u++) {
            if (F[x][u] == INF) continue;
            for (const Tramo& t : m.ady[u]) {
                int nx = x + t.largo;
                if (nx + r[t.destino] > lmax + EPS) continue;  // poda por Lmax
                double c = F[x][u] + solTramo(m, t, x, t0);
                if (c < F[nx][t.destino] - EPS) {
                    F[nx][t.destino] = c;
                    P[nx][t.destino] = {x, u};
                }
            }
        }
    }

    Resultado res;
    int bx = -1;
    for (int x = 0; x <= X; x++) {
        if (F[x][d] < INF && (bx == -1 || F[x][d] < F[bx][d] - EPS)) bx = x;
    }
    if (bx == -1) return res;

    res.hayRuta = true;
    res.sol = F[bx][d];
    res.largo = bx;
    for (int x = bx, u = d; x != -1;) {
        res.ruta.push_back(u);
        auto [px, pu] = P[x][u];
        x = px;
        u = pu;
    }
    reverse(res.ruta.begin(), res.ruta.end());
    return res;
}

// O(Lmax * E) en casi todos los casos, exponencial solo si se usa el respaldo
// Solucion completa, cota, tabla y verificacion de ruta simple
Resultado resolver(const Mapa& m, int o, int d, double alfa, int t0) {
    vector<double> r = dijkstra(m, d);
    if (r[o] == INF) return Resultado();
    double lmax = alfa * r[o];
    Resultado res = programacionDinamica(m, o, d, lmax, t0, r);
    if (res.hayRuta && !esSimple(res.ruta, (int)m.ady.size())) {
        res = fuerzaBruta(m, o, d, lmax, t0, true);
        res.usoRespaldo = true;
    }
    return res;
}

// O(E log V)
// Enfoque ingenuo para comparar, Dijkstra con peso = sol en la franja de salida
// y despues revisar si la ruta cabe en Lmax
Resultado dijkstraIngenuo(const Mapa& m, int o, int d, double alfa, int t0) {
    int n = (int)m.ady.size();
    int k = 0;
    while (k + 1 < (int)m.inicios.size() && m.inicios[k + 1] <= t0) k++;
    vector<double> dist(n, INF);
    vector<int> largo(n, 0), pre(n, -1);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<>> pq;
    dist[o] = 0;
    pq.push({0, o});
    while (!pq.empty()) {
        auto [du, u] = pq.top();
        pq.pop();
        if (du > dist[u]) continue;
        for (const Tramo& t : m.ady[u]) {
            double c = du + t.largo * (1 - t.sombra[k]);
            if (c < dist[t.destino] - EPS) {
                dist[t.destino] = c;
                largo[t.destino] = largo[u] + t.largo;
                pre[t.destino] = u;
                pq.push({c, t.destino});
            }
        }
    }
    Resultado res;
    double lmax = alfa * dijkstra(m, d)[o];
    if (dist[d] == INF || largo[d] > lmax + EPS) return res;
    res.hayRuta = true;
    res.sol = dist[d];
    res.largo = largo[d];
    for (int u = d; u != -1; u = pre[u]) res.ruta.push_back(u);
    reverse(res.ruta.begin(), res.ruta.end());
    return res;
}

// pruebas

string textoRuta(const Mapa& m, const vector<int>& ruta) {
    string s;
    for (size_t i = 0; i < ruta.size(); i++) s += (i ? "-" : "") + m.nombres[ruta[i]];
    return s;
}

int pasadas = 0, fallidas = 0;

void revisar(const string& nombre, bool ok) {
    printf("  [%s] %s\n", ok ? "OK" : "FALLA", nombre.c_str());
    ok ? pasadas++ : fallidas++;
}

void mostrar(const Mapa& m, const Resultado& r) {
    if (!r.hayRuta) {
        printf("    sin ruta\n");
        return;
    }
    printf("    ruta %s, largo %d m, sol %.2f m%s\n", textoRuta(m, r.ruta).c_str(), r.largo, r.sol,
           r.usoRespaldo ? " (respaldo)" : "");
}

// Ejemplo del reporte 1, franjas de 9 h y 13 h
Mapa ejemplo(int& A, int& F) {
    Mapa m;
    m.inicios = {9 * 3600, 13 * 3600};
    A = m.agregarPunto("A");
    int B = m.agregarPunto("B"), C = m.agregarPunto("C");
    int D = m.agregarPunto("D"), E = m.agregarPunto("E");
    F = m.agregarPunto("F");
    m.agregarTramo(A, B, 80, {0.25, 0.25});
    m.agregarTramo(A, C, 60, {0.0, 0.0});
    m.agregarTramo(B, D, 70, {0.3, 0.9});
    m.agregarTramo(C, D, 50, {0.2, 0.2});
    m.agregarTramo(C, E, 90, {1.0, 1.0});
    m.agregarTramo(D, F, 60, {0.5, 0.5});
    m.agregarTramo(E, F, 100, {0.8, 0.8});
    return m;
}

Mapa aleatorio(mt19937& gen, int n, int extra, int franjas) {
    Mapa m;
    for (int k = 0; k < franjas; k++) m.inicios.push_back(9 * 3600 + k * 60);
    for (int i = 0; i < n; i++) m.agregarPunto(to_string(i));
    uniform_int_distribution<int> largo(5, 40);
    uniform_int_distribution<int> s10(0, 10);
    auto sombra = [&]() {
        vector<double> s(franjas);
        for (double& x : s) x = s10(gen) / 10.0;
        return s;
    };
    for (int i = 1; i < n; i++) m.agregarTramo(uniform_int_distribution<int>(0, i - 1)(gen), i, largo(gen), sombra());
    for (int j = 0; j < extra; j++) {
        int u = uniform_int_distribution<int>(0, n - 1)(gen);
        int w = uniform_int_distribution<int>(0, n - 1)(gen);
        if (u != w) m.agregarTramo(u, w, largo(gen), sombra());
    }
    return m;
}

// Cuadricula del tamano aproximado del campus, franjas de 30 min de 7 h a 19 h
Mapa campus(mt19937& gen, int filas, int cols) {
    Mapa m;
    for (int h = 7 * 3600; h < 19 * 3600; h += 1800) m.inicios.push_back(h);
    int franjas = (int)m.inicios.size();
    for (int i = 0; i < filas * cols; i++) m.agregarPunto(to_string(i));
    uniform_int_distribution<int> largo(20, 80);
    uniform_real_distribution<double> s01(0, 1);
    auto sombra = [&]() {
        vector<double> s(franjas);
        for (double& x : s) x = s01(gen);
        return s;
    };
    for (int f = 0; f < filas; f++)
        for (int c = 0; c < cols; c++) {
            int u = f * cols + c;
            if (c + 1 < cols) m.agregarTramo(u, u + 1, largo(gen), sombra());
            if (f + 1 < filas) m.agregarTramo(u, u + cols, largo(gen), sombra());
        }
    return m;
}

int main() {
    int A, F;
    Mapa m = ejemplo(A, F);

    printf("Prueba 1, ejemplo del reporte 1, salida 9 h, alfa 1.3\n");
    Resultado r = resolver(m, A, F, 1.3, 9 * 3600);
    mostrar(m, r);
    revisar("A-C-D-F con 130 m al sol", textoRuta(m, r.ruta) == "A-C-D-F" && fabs(r.sol - 130) < 1e-6);

    printf("Prueba 2, salida 13 h, alfa 1.3\n");
    r = resolver(m, A, F, 1.3, 13 * 3600);
    mostrar(m, r);
    revisar("A-B-D-F con 97 m al sol", textoRuta(m, r.ruta) == "A-B-D-F" && fabs(r.sol - 97) < 1e-6);

    printf("Prueba 3, salida 12 h 58 min 30 s, la ruta cruza de franja\n");
    r = resolver(m, A, F, 1.3, 12 * 3600 + 58 * 60 + 30);
    mostrar(m, r);
    revisar("A-B-D-F con 116.5 m al sol", textoRuta(m, r.ruta) == "A-B-D-F" && fabs(r.sol - 116.5) < 1e-6);

    printf("Prueba 4, salida 13 h, alfa 1.2 (Lmax = 204 m)\n");
    r = resolver(m, A, F, 1.2, 13 * 3600);
    mostrar(m, r);
    revisar("A-C-D-F con 130 m al sol", textoRuta(m, r.ruta) == "A-C-D-F" && fabs(r.sol - 130) < 1e-6);
    Resultado ing = dijkstraIngenuo(m, A, F, 1.2, 13 * 3600);
    printf("    Dijkstra ingenuo\n");
    mostrar(m, ing);
    revisar("el Dijkstra ingenuo no encuentra ruta aunque si existe", !ing.hayRuta);

    printf("Prueba 5, destino aislado\n");
    int G = m.agregarPunto("G");
    r = resolver(m, A, G, 1.3, 9 * 3600);
    mostrar(m, r);
    revisar("responde sin ruta", !r.hayRuta);

    printf("Prueba 6, origen igual a destino\n");
    r = resolver(m, A, A, 1.3, 9 * 3600);
    mostrar(m, r);
    revisar("ruta A con 0 m", r.hayRuta && r.largo == 0 && r.sol == 0);

    printf("Prueba 7, alfa 1.0 solo admite la ruta mas corta\n");
    r = resolver(m, A, F, 1.0, 13 * 3600);
    mostrar(m, r);
    revisar("A-C-D-F con 170 m", textoRuta(m, r.ruta) == "A-C-D-F" && r.largo == 170);

    mt19937 gen(2026);

    printf("Prueba 8, tabla contra busqueda exhaustiva de recorridos (500 grafos)\n");
    int iguales = 0;
    for (int it = 0; it < 500; it++) {
        int n = uniform_int_distribution<int>(4, 8)(gen);
        Mapa g = aleatorio(gen, n, n, 3);
        int o = 0, d = n - 1;
        int t0 = 9 * 3600 + uniform_int_distribution<int>(-30, 150)(gen);
        double alfa = uniform_real_distribution<double>(1.0, 1.6)(gen);
        vector<double> rr = dijkstra(g, d);
        double lmax = alfa * rr[o];
        Resultado a = programacionDinamica(g, o, d, lmax, t0, rr);
        Resultado b = fuerzaBruta(g, o, d, lmax, t0, false);
        if (a.hayRuta == b.hayRuta && fabs(a.sol - b.sol) < 1e-6) iguales++;
    }
    printf("    coinciden %d de 500\n", iguales);
    revisar("la tabla da el mismo sol que la busqueda exhaustiva", iguales == 500);

    printf("Prueba 9, solucion completa contra busqueda exhaustiva de rutas simples (3000 grafos)\n");
    iguales = 0;
    int respaldos = 0;
    for (int it = 0; it < 3000; it++) {
        int n = uniform_int_distribution<int>(4, 9)(gen);
        Mapa g = aleatorio(gen, n, n, 3);
        int o = 0, d = n - 1;
        int t0 = 9 * 3600 + uniform_int_distribution<int>(-30, 150)(gen);
        double alfa = uniform_real_distribution<double>(1.0, 1.6)(gen);
        Resultado a = resolver(g, o, d, alfa, t0);
        Resultado b = fuerzaBruta(g, o, d, alfa * dijkstra(g, d)[o], t0, true);
        if (a.usoRespaldo) respaldos++;
        if (a.hayRuta == b.hayRuta && fabs(a.sol - b.sol) < 1e-6 && esSimple(a.ruta, n)) iguales++;
    }
    printf("    coinciden %d de 3000, respaldo usado en %d casos (%.2f %%)\n", iguales, respaldos,
           100.0 * respaldos / 3000);
    revisar("la solucion completa da el optimo entre rutas simples", iguales == 3000);

    printf("Prueba 10, tiempo en una cuadricula de 20 x 25 puntos (500 puntos, 24 franjas)\n");
    Mapa c = campus(gen, 20, 25);
    int consultas = 200, conRespaldo = 0;
    double peorTabla = 0, sumaTabla = 0, peorTotal = 0;
    for (int q = 0; q < consultas; q++) {
        int o = uniform_int_distribution<int>(0, 499)(gen);
        int d = uniform_int_distribution<int>(0, 499)(gen);
        int t0 = uniform_int_distribution<int>(7 * 3600, 18 * 3600)(gen);

        auto ini = chrono::steady_clock::now();
        vector<double> rr = dijkstra(c, d);
        programacionDinamica(c, o, d, 1.3 * rr[o], t0, rr);
        double ms = chrono::duration<double, milli>(chrono::steady_clock::now() - ini).count();
        sumaTabla += ms;
        peorTabla = max(peorTabla, ms);

        ini = chrono::steady_clock::now();
        Resultado rq = resolver(c, o, d, 1.3, t0);
        ms = chrono::duration<double, milli>(chrono::steady_clock::now() - ini).count();
        peorTotal = max(peorTotal, ms);
        if (rq.usoRespaldo) conRespaldo++;
    }
    printf("    tabla sola, promedio %.1f ms y peor %.1f ms en %d consultas\n", sumaTabla / consultas,
           peorTabla, consultas);
    revisar("la tabla responde en menos de 1 s", peorTabla < 1000);
    printf("    solucion completa, respaldo en %d de %d consultas, peor %.1f ms\n", conRespaldo, consultas,
           peorTotal);
    printf("    (pendiente, el respaldo exhaustivo no escala en mapas grandes)\n");

    printf("\nResumen, %d pruebas OK y %d fallidas\n", pasadas, fallidas);
    return fallidas == 0 ? 0 : 1;
}
