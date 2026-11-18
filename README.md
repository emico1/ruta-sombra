# Ruta peatonal con menor exposición al sol

Situación problema de Análisis y diseño de algoritmos avanzados.

Encuentra la ruta entre dos puntos del campus que recibe la menor cantidad de sol sin caminar más que α veces la ruta más corta. La sombra de cada tramo cambia por franja horaria, y con una velocidad de 1.25 m/s un tramo que cruza el cambio de franja se parte en dos.

## Técnica

Programación dinámica sobre el estado (metros caminados, punto), con poda por Lmax usando un Dijkstra desde el destino. Si la ruta reconstruida repite puntos se usa una búsqueda exhaustiva con poda como respaldo.

- Tabla, O(Lmax · E · H) en tiempo y O(Lmax · V) en memoria
- Respaldo, exponencial (solo cuando la ruta de la tabla repite puntos)

## Compilar y correr

```
g++ -std=c++17 -O2 -Wall -Wextra -o ruta_sombra ruta_sombra.cpp
./ruta_sombra
```

El programa corre las 10 pruebas del reporte de avance y al final imprime cuántas pasaron.

| # | Prueba |
|---|---|
| 1 a 3 | Ejemplo del reporte 1 con salida a las 9 h, 13 h y 12 h 58 min 30 s |
| 4 | α = 1.2 contra un Dijkstra ingenuo |
| 5 a 7 | Destino aislado, origen igual a destino, α = 1.0 |
| 8 | 500 grafos aleatorios, tabla contra fuerza bruta de recorridos |
| 9 | 3,000 grafos aleatorios, solución completa contra fuerza bruta de rutas simples |
| 10 | Tiempo en una cuadrícula de 500 puntos con 24 franjas |

Los grafos aleatorios usan la semilla 2026, así que los resultados se repiten en cada corrida.