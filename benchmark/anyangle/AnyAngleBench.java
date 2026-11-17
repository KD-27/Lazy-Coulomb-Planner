// Times grid any-angle and grid planners from the Any-Angle-Pathfinding collection
// (github.com/Ohohcakester/Any-Angle-Pathfinding; Anya16 is Harabor's code) on
// LCP's benchmark maps and start/goal pairs.
//
// Usage: java -cp <build> AnyAngleBench <grid.txt> <queries.csv> <out.csv> <repeats> <algo>...
// grid.txt:    "W H" then H lines of W chars, '1' = blocked tile; line 0 is y = 0.
// queries.csv: pair,sx,sy,gx,gy in grid vertex coordinates (integers).
// Algorithms:  Anya16, JPS, JPS_PS (post-smoothed), AStar, AStar_PS, Theta, LazyTheta.
// Every query is run once untimed (JIT warm-up), then `repeats` timed runs; the
// median time is reported. Lengths are in grid units (multiply by the resolution).
import algorithms.AStar;
import algorithms.BasicThetaStar;
import algorithms.JumpPointSearch;
import algorithms.LazyThetaStar;
import algorithms.PathFindingAlgorithm;
import algorithms.anya16.Anya16;
import grid.GridGraph;
import java.io.*;
import java.nio.file.*;
import java.util.*;

public class AnyAngleBench {
    interface Factory { PathFindingAlgorithm make(GridGraph g, int sx, int sy, int ex, int ey); }

    static Factory factory(String name) {
        switch (name) {
            case "Anya16": return Anya16::new;
            case "JPS": return JumpPointSearch::new;
            case "JPS_PS": return JumpPointSearch::postSmooth;
            case "AStar": return AStar::new;
            case "AStar_PS": return AStar::postSmooth;
            case "Theta": return BasicThetaStar::new;
            case "LazyTheta": return LazyThetaStar::new;
            default: throw new IllegalArgumentException("unknown algorithm " + name);
        }
    }

    static double length(int[][] path) {
        double l = 0;
        for (int i = 0; i + 1 < path.length; i++) {
            l += Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1]);
        }
        return l;
    }

    public static void main(String[] a) throws Exception {
        List<String> lines = Files.readAllLines(Paths.get(a[0]));
        String[] wh = lines.get(0).trim().split("\\s+");
        int w = Integer.parseInt(wh[0]), h = Integer.parseInt(wh[1]);
        GridGraph g = new GridGraph(w, h);
        for (int y = 0; y < h; y++) {
            String row = lines.get(1 + y);
            for (int x = 0; x < w; x++) g.setBlocked(x, y, row.charAt(x) == '1');
        }
        List<int[]> q = new ArrayList<>();
        for (String s : Files.readAllLines(Paths.get(a[1]))) {
            if (s.startsWith("pair")) continue;
            String[] f = s.split(",");
            q.add(new int[]{Integer.parseInt(f[0]), Integer.parseInt(f[1]), Integer.parseInt(f[2]),
                            Integer.parseInt(f[3]), Integer.parseInt(f[4])});
        }
        int repeats = Integer.parseInt(a[3]);
        try (PrintWriter out = new PrintWriter(new FileWriter(a[2]))) {
            out.println("pair,algo,valid_endpoints,found,time_ms,time_min_ms,time_max_ms,length_cells,waypoints");
            for (int k = 4; k < a.length; k++) {
                Factory f = factory(a[k]);
                for (int[] p : q) {
                    boolean ends = g.isUnblockedCoordinate(p[1], p[2]) && g.isUnblockedCoordinate(p[3], p[4]);
                    if (!ends) {
                        out.printf(Locale.ROOT, "%d,%s,0,0,nan,nan,nan,nan,0%n", p[0], a[k]);
                        continue;
                    }
                    PathFindingAlgorithm alg = f.make(g, p[1], p[2], p[3], p[4]);
                    alg.computePath();  // warm-up, untimed
                    double[] t = new double[repeats];
                    int[][] path = null;
                    for (int r = 0; r < repeats; r++) {
                        alg = f.make(g, p[1], p[2], p[3], p[4]);
                        long t0 = System.nanoTime();
                        alg.computePath();
                        t[r] = (System.nanoTime() - t0) / 1e6;
                        if (r == 0) path = alg.getPath();
                    }
                    Arrays.sort(t);
                    boolean found = path != null && path.length >= 2;
                    out.printf(Locale.ROOT, "%d,%s,1,%d,%.4f,%.4f,%.4f,%s,%d%n", p[0], a[k], found ? 1 : 0,
                               t[repeats / 2], t[0], t[repeats - 1],
                               found ? String.format(Locale.ROOT, "%.3f", length(path)) : "nan",
                               found ? path.length : 0);
                }
                out.flush();
            }
        }
    }
}
