package frc.robot.math.geometry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;

public class DelaunayTriangulation {


    public final Triangle2d[] triangles;

    public final int[] indices;

    public DelaunayTriangulation(Translation2d[] points) {
        var triangulation = triangulate(Arrays.asList(points));
        this.triangles = new Triangle2d[triangulation.size()];
        this.indices = new int[triangulation.size() * 3];
        for (int i = 0; i < triangulation.size(); i++) {
            var tri = triangulation.get(i);
            this.triangles[i] = new Triangle2d(points[tri.i1], points[tri.i2], points[tri.i3]);
            this.indices[i * 3 + 0] = tri.i1;
            this.indices[i * 3 + 1] = tri.i2;
            this.indices[i * 3 + 2] = tri.i3;
        }

        System.out.println("triangulation: ");
        System.out.println("  triangles (n = " + triangles.length + "):");
        for (var tri : triangles) {
            System.out.println("    - " + tri.a);
            System.out.println("      " + tri.b);
            System.out.println("      " + tri.c);
        }
    }

    private static class Edge {
        public final int i1;
        public final int i2;
        public boolean isBad = false;

        public Edge(int i1, int i2) {
            this.i1 = i1;
            this.i2 = i2;
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof Edge))
                return false;
            Edge e = (Edge) obj;
            return (i1 == e.i1 && i2 == e.i2) || (i1 == e.i2 && i2 == e.i1);
        }

        @Override
        public int hashCode() {
            return Integer.hashCode(i1) + Integer.hashCode(i2);
        }
    }

    private static class Triangle {
        public final int i1;
        public final int i2;
        public final int i3;
        public boolean isBad = false;

        private final List<Translation2d> vertices;

        public Triangle(int i1, int i2, int i3, List<Translation2d> vertices) {
            this.i1 = i1;
            this.i2 = i2;
            this.i3 = i3;
            this.vertices = vertices;
        }

        public boolean containsVertex(int index) {
            return i1 == index || i2 == index || i3 == index;
        }

        public boolean circumcircleContains(int i) {
            Translation2d v = vertices.get(i);
            Translation2d a = vertices.get(i1);
            Translation2d b = vertices.get(i2);
            Translation2d c = vertices.get(i3);

            double ab = a.getSquaredNorm();
            double cd = b.getSquaredNorm();
            double ef = c.getSquaredNorm();

            double ax = a.getX();
            double ay = a.getY();
            double bx = b.getX();
            double by = b.getY();
            double cx = c.getX();
            double cy = c.getY();

            double circum_x = (ab * (cy - by) + cd * (ay - cy) + ef * (by - ay))
                / (ax * (cy - by) + bx * (ay - cy) + cx * (by - ay));
            double circum_y = (ab * (cx - bx) + cd * (ax - cx) + ef * (bx - ax))
                / (ay * (cx - bx) + by * (ax - cx) + cy * (bx - ax));

            Translation2d circum = new Translation2d(circum_x / 2.0, circum_y / 2.0);
            double circum_radius = a.getSquaredDistance(circum);
            double dist = v.getSquaredDistance(circum);

            boolean res = dist <= circum_radius;

            return res;
        }
    }

    private static List<Triangle> triangulate(List<Translation2d> in_points) {
        List<Translation2d> vertices = new ArrayList<>(in_points);
        List<Triangle> triangles = new ArrayList<>();

        // Bounding box
        double minX = in_points.get(0).getX();
        double minY = in_points.get(0).getY();
        double maxX = minX;
        double maxY = minY;

        for (Translation2d p : in_points) {
            minX = Math.min(minX, p.getX());
            minY = Math.min(minY, p.getY());
            maxX = Math.max(maxX, p.getX());
            maxY = Math.max(maxY, p.getY());
        }

        double dx = maxX - minX;
        double dy = maxY - minY;
        double deltaMax = Math.max(dx, dy);
        double midx = (minX + maxX) / 2;
        double midy = (minY + maxY) / 2;

        // Add super triangle vertices
        int super1 = vertices.size();
        vertices.add(new Translation2d(midx - 20 * deltaMax, midy - deltaMax));

        int super2 = vertices.size();
        vertices.add(new Translation2d(midx, midy + 20 * deltaMax));

        int super3 = vertices.size();
        vertices.add(new Translation2d(midx + 20 * deltaMax, midy - deltaMax));

        triangles.add(new Triangle(super1, super2, super3, vertices));

        // Insert each Translation2d
        for (int i = 0; i < in_points.size(); i++) {

            List<Edge> polygon = new ArrayList<>();

            for (Triangle t : triangles) {
                if (t.circumcircleContains(i)) {
                    t.isBad = true;
                    polygon.add(new Edge(t.i1, t.i2));
                    polygon.add(new Edge(t.i2, t.i3));
                    polygon.add(new Edge(t.i3, t.i1));
                }
            }

            triangles.removeIf(t -> t.isBad);

            for (int e1i = 0; e1i < polygon.size(); e1i++) {
                var e1 = polygon.get(e1i);
                for (int e2i = e1i + 1; e2i < polygon.size(); e2i++) {
                    var e2 = polygon.get(e2i);
                    if (e1.equals(e2)) {
                        e1.isBad = true;
                        e2.isBad = true;
                    }
                }
            }

            polygon.removeIf(e -> e.isBad);

            for (var e : polygon) {
                triangles.add(new Triangle(e.i1, e.i2, i, vertices));
            }
        }

        // Remove triangles connected to super triangle
        triangles.removeIf(
            t -> t.containsVertex(super1) || t.containsVertex(super2) || t.containsVertex(super3));

        return triangles;
    }

}
