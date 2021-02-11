package ch.fridolinsrobotik.utilities;

public class Vector2d extends edu.wpi.first.wpilibj.drive.Vector2d {
    public Vector2d() {
        super();
    }

    public Vector2d(double x, double y) {
        super(x, y);
    }

    /**
     * @param vec Vector with which to perform cross product.
     * @return Cross product of this vector and "vec" (this x vec).
     */
    public double cross(Vector2d vec) {
        return (x * vec.y) - (y * vec.x);
    }

    /**
     * @param dotProduct The dot product you would get by taking the dot product of the result and this vector.
     * @return The tow vectors with a dot product of "dotProduct". Both vectors in the pair are normalized.
     * @throws ArithmeticException If the vector isn't normalized or "dotProduct" is not a value between -1 and 1.
     */
    public Pair<Vector2d, Vector2d> inverseDot(double dotProduct) {
        if (magnitude() != 1)
            throw new ArithmeticException(
                    "The magnitude of the vector has to be 1 to perform an inverse dot product. Magnitude is: "
                            + magnitude());

        if (!(dotProduct <= 1 && dotProduct >= -1))
            throw new ArithmeticException("The dotProduct has to be between -1 and 1, since the dot product of two normalized vector can only give you a number between -1 and 1.");

        Vector2d firstSolution = new Vector2d();
        firstSolution.x = (-(Math.sqrt(x * x + y * y - dotProduct * dotProduct) * y - x * dotProduct))
                / (x * x + y * y);
        firstSolution.y = (x * Math.sqrt(x * x + y * y - dotProduct * dotProduct) + y * dotProduct) / (x * x + y * y);

        Vector2d secondSolution = new Vector2d();
        secondSolution.x = (Math.sqrt(x * x + y * y - dotProduct * dotProduct) * y + x * dotProduct) / (x * x + y * y);
        secondSolution.y = (-(x * Math.sqrt(x * x + y * y - dotProduct * dotProduct) - y * dotProduct))
                / (x * x + y * y);

        return new Pair<Vector2d, Vector2d>(firstSolution, secondSolution);
    }

    /**
     * Normalizes the vector, the magnitude will be 1 after this operation.
     *
     * @throws ArithmeticException If the magnitude of the the vector is 0;
     */
    public void normalize() {
        if (magnitude() == 0)
            throw new ArithmeticException("The magnitude of the vector mustn't be 0 to normalize it.");
        x /= magnitude();
        y /= magnitude();
    }

    /**
     * Adds the vector "vec" to this vector.
     *
     * @param vec The vector to be added.
     */
    public void add(Vector2d vec) {
        x += vec.x;
        y += vec.y;
    }

    /**
     * Subtract the vector "vec" from this vector.
     *
     * @param vec The vector to be subtracted.
     */
    public void sub(Vector2d vec) {
        x -= vec.x;
        y -= vec.y;
    }

    /**
     * Multiplies x and y by "scalar".
     *
     * @param scalar The factor to be multiplied by.
     */
    public void mult(double scalar) {
        x *= scalar;
        y *= scalar;
    }

    /**
     * Divides both x and y by "scalar".
     *
     * @param scalar The divisor.
     */
    public void div(double scalar) {
        mult(1 / scalar);
    }

    /**
     * @param theta The angle in the polar space, in radians
     * @param r     The lenght of the vector both in polar and cartesian space
     * @return A new 2d vector with an angle to the x axis of "theta" and a length of "r"
     */
    public static Vector2d fromPolar(double theta, double r) {
        return new Vector2d(Math.cos(theta) * r, Math.sin(theta) * r);
    }

    /**
     * @param angle The angle of the new vector in radians
     * @return A new normalized 2d vector with an angle of "angle" to the x axis
     */
    public static Vector2d fromRad(double angle) {
        return fromPolar(angle, 1.0);
    }

    public String toString() {
        return String.format("Vector2d: [%f, %f]", x, y);
    }
}