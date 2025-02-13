package org.firstinspires.ftc.teamcode.coyote.geometry;

public class Pose extends Point {

    public double a;

    public Pose(double x, double y, double a) {
        this.x = x;
        this.y = y;
        this.a = (a > (Math.PI * 2)) ? (a - (Math.PI * 2)) : a;;
    }

    public void set_x(double x) {
        this.x = x;
    }
    public void set_y(double y) {
        this.y = y;
    }
    public void set_a(double a) {
        this.a = a;
    }

    public Pose(double x, double y) {
        this(x, y, 0);
    }

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(Pose other) {
        this(other.x, other.y, other.a);
    }

    public Pose(Point point) {
        this(point.x, point.y);
    }

    public Pose clone() {
        return new Pose(this);
    }

    public Pose copy(Pose other) {
        this.x = other.x;
        this.y = other.y;
        this.a = other.a;
        return this;
    }

    public Point to_point() {
        return new Point(this);
    }
}
