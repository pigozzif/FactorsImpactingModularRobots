package it.units.erallab.factors;

import it.units.erallab.hmsrobots.util.Grid;
import org.apache.commons.math3.util.Pair;

import java.util.*;
import java.util.stream.Collectors;


public class ExtractFeatures {

    public static void prettyPrintGrid(Grid<Boolean> g) {
        for (int i = 0; i < g.getW(); ++i) {
            int finalI = i;
            System.out.println("[" + g.stream().filter(e -> e.getY() == finalI).map(e -> String.valueOf(e.getValue())).collect(Collectors.joining(",")) + "]");
        }
    }

    public static double shapeElongation(Grid<Boolean> posture, int n) {
        if (posture.values().stream().noneMatch(e -> e)) {
            throw new IllegalArgumentException("Grid is empty");
        }
        else if (n <= 0) {
            throw new IllegalArgumentException(String.format("Non-positive number of directions provided: %d", n));
        }
        List<Pair<Integer, Integer>> coordinates = posture.stream().filter(Grid.Entry::getValue).map(e -> new Pair<>(e.getX(), e.getY())).collect(Collectors.toList());
        List<Double> diameters = new ArrayList<>();
        for (int i = 0; i < n; ++i) {
            double theta = (2 * i * Math.PI) / n;
            List<Pair<Double, Double>> rotatedCoordinates = coordinates.stream().map(p -> new Pair<>(p.getFirst() * Math.cos(theta) - p.getSecond() * Math.sin(theta), p.getFirst() * Math.sin(theta) + p.getSecond() * Math.cos(theta))).collect(Collectors.toList());
            double minX = rotatedCoordinates.stream().min(Comparator.comparingDouble(Pair::getFirst)).get().getFirst();
            double maxX = rotatedCoordinates.stream().max(Comparator.comparingDouble(Pair::getFirst)).get().getFirst();
            double minY = rotatedCoordinates.stream().min(Comparator.comparingDouble(Pair::getSecond)).get().getSecond();
            double maxY = rotatedCoordinates.stream().max(Comparator.comparingDouble(Pair::getSecond)).get().getSecond();
            double sideX = maxX - minX + 1;
            double sideY = maxY - minY + 1;
            diameters.add(Math.min(sideX, sideY) / Math.max(sideX, sideY));
        }
        return 1.0 - Collections.min(diameters);
    }

    public static double shapeCompactness(Grid<Boolean> posture) {
        // approximate convex hull
        Grid<Boolean> convexHull = Grid.create(posture.getW(), posture.getH(), posture::get);
        boolean none = false;
        // loop as long as there are false cells have at least five of the eight Moore neighbors as true
        while (!none) {
            none = true;
            for (Grid.Entry<Boolean> entry : convexHull) {
                if (convexHull.get(entry.getX(), entry.getY())) {
                    continue;
                }
                int currentX = entry.getX();
                int currentY = entry.getY();
                int adjacentCount = 0;
                // count how many of the Moore neighbors are true
                for (int i : new int[]{1, -1}) {
                    int neighborX = currentX;
                    int neighborY = currentY + i;
                    if (0 <= neighborY && neighborY < convexHull.getH() && convexHull.get(neighborX, neighborY)) {
                        adjacentCount += 1;
                    }
                    neighborX = currentX + i;
                    neighborY = currentY;
                    if (0 <= neighborX && neighborX < convexHull.getW() && convexHull.get(neighborX, neighborY)) {
                        adjacentCount += 1;
                    }
                    neighborX = currentX + i;
                    neighborY = currentY + i;
                    if (0 <= neighborX && 0 <= neighborY && neighborX < convexHull.getW() && neighborY < convexHull.getH() && convexHull.get(neighborX, neighborY)) {
                        adjacentCount += 1;
                    }
                    neighborX = currentX + i;
                    neighborY = currentY - i;
                    if (0 <= neighborX && 0 <= neighborY && neighborX < convexHull.getW() && neighborY < convexHull.getH() && convexHull.get(neighborX, neighborY)) {
                        adjacentCount += 1;
                    }
                }
                // if at least five, fill the cell
                if (adjacentCount >= 5) {
                    convexHull.set(entry.getX(), entry.getY(), true);
                    none = false;
                }
            }
        }
        // compute are ratio between convex hull and posture
        int nVoxels = (int) posture.count(e -> e);
        int nConvexHull = (int) convexHull.count(e -> e);
        // -> 0.0 for less compact shapes, -> 1.0 for more compact shapes
        return (double) nVoxels / nConvexHull;
    }

}
