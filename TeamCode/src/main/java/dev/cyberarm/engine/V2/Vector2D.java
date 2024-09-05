package dev.cyberarm.engine.V2;

public class Vector2D {
    double x, y;
    public Vector2D() {
        this.x = 0;
        this.y = 0;
    }

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public Vector2D plus(Vector2D other) {
        return new Vector2D(
                this.x + other.x(),
                this.y + other.y()
        );
    }

    public Vector2D plus(double other) {
        return new Vector2D(
                this.x + other,
                this.y + other
        );
    }

    public Vector2D minus(Vector2D other) {
        return new Vector2D(
                this.x - other.x(),
                this.y - other.y()
        );
    }

    public Vector2D minus(double other) {
        return new Vector2D(
                this.x - other,
                this.y - other
        );
    }

    public Vector2D multiply(Vector2D other) {
        return new Vector2D(
                this.x * other.x(),
                this.y * other.y()
        );
    }

    public Vector2D multiply(double other) {
        return new Vector2D(
                this.x * other,
                this.y * other
        );
    }

    public Vector2D divide(Vector2D other) {
        return new Vector2D(
                (this.x == 0 ? 0.0 : this.x / other.x()),
                (this.y == 0 ? 0.0 : this.y / other.y())
        );
    }

    public Vector2D divide(double other) {
        return new Vector2D(
                (this.x == 0 ? 0.0 : this.x / other),
                (this.y == 0 ? 0.0 : this.y / other)
        );
    }

    public double magnitude() {
        return Math.sqrt((this.x * this.x) + (this.y * this.y));
    }

    public Vector2D normalize() {
        double mag = magnitude();

        return divide(mag);
    }

    public double distance(Vector2D other) {
        return Math.sqrt((this.x - other.x) * (this.x - other.x) + (this.y - other.y) * (this.y - other.y));
    }

    public Vector2D inverse() {
        return new Vector2D(1.0 / this.x, 1.0 / this.y);
    }

    public double dot(Vector2D other) {
        double product = 0.0;
        product += this.x * other.x;
        product += this.y * other.y;

        return product;
    }

    public double sum() {
        return this.x + this.y;
    }
}
