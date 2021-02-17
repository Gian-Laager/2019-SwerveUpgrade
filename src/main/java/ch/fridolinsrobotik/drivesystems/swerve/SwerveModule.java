/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import edu.wpi.first.wpilibj.Sendable;

import java.util.Vector;

import org.opencv.core.Algorithm;

import ch.fridolinsrobotik.utilities.Timer;
import ch.fridolinsrobotik.utilities.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * A class for representing a Swerve module.
 * 
 * A Swerve module consists out of two motors. One motor rotates the wheel
 * around its Z-axis to steer and the other around its Y-axis to drive.
 * 
 * <p>
 * This library uses the NED axes convention (North-East-Down as external
 * reference in the world frame):
 * https://upload.wikimedia.org/wikipedia/commons/2/2f/RPY_angles_of_airplanes.png.
 * </p>
 * <p>
 * The positive X axis points ahead, the positive Y axis points right, and the
 * positive Z axis points down. Rotations follow the right-hand rule, so
 * clockwise rotation around the Z axis is positive.
 * </p>
 */
public abstract class SwerveModule implements Sendable {

    // private final SendableImpl m_sendableImpl;

    protected double steeringPulsesPerRotation = 1;
    protected double drivingPulsesPerRotation = 1;
    protected Translation2d mountingPoint = new Translation2d(0, 0);
    protected Translation2d naturalRotateVector = new Translation2d(0, 0);
    protected double driveSpeed = 0;
    protected double driveInverted = 1.0;
    protected int steeringPosition = 0;

    public SwerveModule(Translation2d mountingPoint, double steeringPulsesPerRotation,
            double drivingPulsesPerRotation) {
        SendableRegistry.addLW(this, "SwerveModule");
        // SendableRegistry.setName(this, "SwerveModule");
        verify(mountingPoint, steeringPulsesPerRotation, drivingPulsesPerRotation);
        this.mountingPoint = mountingPoint;
        this.steeringPulsesPerRotation = steeringPulsesPerRotation;
        this.drivingPulsesPerRotation = drivingPulsesPerRotation;
    }

    /**
     * Verifies that mounting point is nonnull, throwing a NullPointerException if
     * it is. The method also verifies that the mounting point is not the origin
     * where a swerve module has not a natural rotating vector.
     * 
     * Verifies thath pulses per rotation (PPR) are not null to prevent division by
     * 0 errors in {@link SwerveModule#convertEncoderPulsesToRadians(int, int)}.
     * 
     * @throws NullPointerException     if the mounting point is null
     * @throws IllegalArgumentException if the mounting point is the origin or if
     *                                  any PPR is 0
     */
    private void verify(Translation2d mountingPoint, double steeringPulsesPerRotation,
            double drivingPulsesPerRotation) {
        if (mountingPoint != null && mountingPoint.getNorm() != 0 && steeringPulsesPerRotation != 0
                && drivingPulsesPerRotation != 0) {
            return;
        }

        if (mountingPoint == null) {
            throw new NullPointerException("Mounting point is missing");
        }

        StringBuilder sb = new StringBuilder();
        if (mountingPoint.getNorm() == 0) {
            sb.append("Mounting point must not be the origin. ");
        }

        if (steeringPulsesPerRotation == 0) {
            sb.append("Steering PPR must not be 0. ");
        }

        if (drivingPulsesPerRotation == 0) {
            sb.append("Driving PPR must not be 0");
        }

        throw new IllegalArgumentException(sb.toString());
    }

    public void setInvertedDrive(boolean inverted) {
        if (inverted) {
            this.driveInverted = -1.0;
        } else {
            this.driveInverted = 1.0;
        }
    }

    public static enum RotationDirection {
        Clockwise, CounterClockwise
    }

    public RotationDirection getRotationDirection() {
        if (convertEncoderPulsesToRadians(getSteeringPosition(), getSteeringPulsesPerRotation())
                - getSteeringAngle() > 0)
            return RotationDirection.CounterClockwise;
        return RotationDirection.Clockwise;
    }

    public void invertRotationDirection() {
        double diriveDirection = -1;
        setDriveSpeedVelocity(getDriveSpeedVelocity() * diriveDirection);

        double invertedAngle = ((Math.PI * 2) - getAngleToSteer()) % (Math.PI * 2);
        double steeringDirection = Math.signum(Vector2d.fromRad(getSteeringAngle()).cross(targetVector));
        invertedAngle *= steeringDirection;
        setSteeringPosition((int) getSteeringEncoderPulses()
                + convertRadiansToEncoderPulses(invertedAngle, getSteeringPulsesPerRotation()));
    }

    /**
     * @return The target vector wich the module trys to achive, lenght equals
     *         module speed in m/s
     */
    public Vector2d getTargetVector() {
        return Vector2d.fromPolar(angleToSteer, getDriveSpeedVelocity());
    }

    public double getAngleToSteer() {
        return angleToSteer;
    }

    protected Vector2d wheelVector;
    protected Vector2d lastTargetVector = null;
    private double angleToSteer = 0.0;
    private Vector2d targetVector = new Vector2d();

    /**
     * Drive calculation method for Serve platform.
     *
     * <p>
     * Angles are measured clockwise from the positive X axis. The robot's speed is
     * independent from its angle or rotation rate.
     *
     * @param ySpeed    The robot's speed along the Y axis [-1.0..1.0]. Right is
     *                  positive.
     * @param xSpeed    The robot's speed along the X axis [-1.0..1.0]. Forward is
     *                  positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
     *                  Clockwise is positive.
     */
    protected void calculateSwerveMovement(double speedMetersPerSecond, Rotation2d angle) {
        wheelVector = Vector2d.fromPolar(getSteeringAngle(), speedMetersPerSecond);
        Vector2d normalizedWheelVecotr = Vector2d.fromRad(getSteeringAngle());
        targetVector = Vector2d.fromRad(angle.getRadians());
        // System.out.println(String.format(
        // "Limited dot product: %f, velocity: %f, actual target vector: %s, limited
        // target vector: %s, wheel vector: %s",
        // getLimitedDotProduct(speedMeterPerSecond), speedMeterPerSecond,
        // targetVector.toString(),
        // getLimitedSteeringVector(normalizedWheelVecotr, targetVector,
        // speedMeterPerSecond).toString(),
        // wheelVector.toString()));
        System.out.print("Actual target vector: " + targetVector.toString());

        Vector2d limitedTargetVector = getLimitedSteeringVector(normalizedWheelVecotr, targetVector,
                getDriveSpeedVelocity());
        if (lastTargetVector != null)
            System.out.print(String.format(", limited target vector: %s, wheel vector: %s, velocity: %f",
                    limitedTargetVector.toString(), normalizedWheelVecotr.toString(),
                    limitedTargetVector.dot(lastTargetVector), getDriveVelocityFromEncoder()));

        /*
         * Angle between wheel vector and target vector. Only the target vector's
         * magnitude is needed, since the wheel vector's magnitude is always 1.
         */
        angleToSteer = Math.acos(limitedTargetVector.dot(normalizedWheelVecotr));
        double steeringDirection = Math.signum(normalizedWheelVecotr.cross(targetVector));

        double driveDirection;
        /* if steering angle is bigger than 90' the opposite side (-180') is faster */
        if (angleToSteer > Math.PI / 2) {
            angleToSteer -= Math.PI;
            driveDirection = -1;
        } else {
            driveDirection = 1;
        }

        angleToSteer *= steeringDirection;

        setSteeringPosition((int) getSteeringEncoderPulses()
                + convertRadiansToEncoderPulses(angleToSteer, getSteeringPulsesPerRotation()));
        System.out.println(", target angle: " + Math.toDegrees(getSteeringAngle()) % 360.0);
        setDriveSpeedVelocity(driveDirection * driveInverted * speedMetersPerSecond);
        lastTargetVector = limitedTargetVector;
    }

    /**
     * <p>
     * Default value when {@link #getLoopTime()} hasn't been called.
     * </p>
     * <b>DO NOT CHANGE</b> the function {@link #modifiedGauseFunction(double)} has
     * been modified for this exact time.
     */

    private static final double defaultLoopTime = 0.02;

    /**
     * factor to streche Gause curve in x direction
     */
    protected static final double gauseFactor = -Math.log(Math.PI / 4.59678e8);

    /**
     * y offset of the Gause curve
     */
    protected static final double gauseOffset = 1;

    private static double modifiedGauseFunction(double x) {
        return -Math.exp(-(x * x) * gauseFactor) + gauseOffset;
    }

    private Timer loopTimeLimitedDotProduct = new Timer();

    /**
     * <b>Note</b>: This function should only be used with the modified gause
     * function since it returns {@link #defaultLoopTime} when the function hasn't
     * been called yet.
     * 
     * @return The time that has passed sinse the function has been called in
     *         seconds
     */
    private double getLoopTime() {
        double time = defaultLoopTime;
        if (loopTimeLimitedDotProduct.getLastStarted() != -1)
            time = loopTimeLimitedDotProduct.getLastStarted();
        loopTimeLimitedDotProduct.start();
        return time;
    }

    public double getLimitedDotProduct(double velocity) {
        double result = MathUtil.clamp(modifiedGauseFunction(velocity)/* / (getLoopTime() / defaultLoopTime) */, -1.0,
                1.0);
        System.out.print(", limited dot product: " + result);
        return result;
        // return MathUtil.clamp(modifiedGauseFunction(velocity) / (getLoopTime() /
        // defaultLoopTime), -1.0, 1.0);
        // return modifiedGauseFunction(velocity);
    }

    protected abstract Vector2d getLimitedSteeringVector(Vector2d moduleRotation, Vector2d targetRotation,
            double velocity);

    /**
     * @return Returns the rotation of the wheel measured by the encoder
     */
    public abstract Rotation2d getDriveAngleFromEncoder();

    /**
     * @return Velocity of the wheel measured from the encoder in meters per second
     */
    public abstract double getDriveVelocityFromEncoder();

    /**
     * Feeds the calculated swerve movement values into the motor controllers.
     */
    public abstract void executeSwerveMovement();

    /**
     * Is used to check the homing sensor from different motor controllers
     * 
     * @return True when the homing sensor is active.
     */
    public abstract boolean homingSensorActive();

    public Vector2d getTargetRotation() {
        return targetVector;
    }

    /**
     * Sets the steering motor to velocity control mode instead of position control
     * mode. Can be used to home the steering wheel independent of resistance
     * applied to the wheel.
     * 
     * @param unitsPer100ms velocity in units per 100ms.
     */
    public abstract void setSteeringVelocity(int unitsPer100ms);

    public abstract void stopMotors();

    public abstract void resetSteeringEncoder();

    public abstract void resetDrivingEncoder();

    /**
     * @param steeringPulsesPerRotation Pulses per rotation of the wheel for a full
     *                                  turn around the wheel's Z-axis
     */
    public void setSteeringPulsesPerRotation(int steeringPulsesPerRotation) {
        this.steeringPulsesPerRotation = steeringPulsesPerRotation;
    }

    /**
     * @param drivingPulsesPerRotation Pulses per rotation of the wheel for a full
     *                                 turn around the wheel's Y-axis
     */
    public void setDrivingPulsesPerRotation(int drivingPulsesPerRotation) {
        this.drivingPulsesPerRotation = drivingPulsesPerRotation;
    }

    /**
     * Sets the driving speed of the Swerve module. Can be used to adjust the speed
     * of it.
     * 
     * @param driveSpeedPercentage Motor speed in percentage [-1..1].
     */
    protected void setDriveSpeedVelocity(double driveSpeedPercentage) {
        this.driveSpeed = driveSpeedPercentage;
    }

    /**
     * Sets the steering position of the Swerve module. Can be used to adjust the
     * position of it.
     * 
     * @param steeringPosition Absolute encoder position of the steering wheel.
     */
    protected void setSteeringPosition(int steeringPosition) {
        this.steeringPosition = steeringPosition;
    }

    /**
     * @return Pulses per rotation of the wheel for a full turn around the wheel's
     *         Z-axis
     */
    public double getSteeringPulsesPerRotation() {
        return this.steeringPulsesPerRotation;
    }

    /**
     * @return Pulses per rotation of the wheel for a full turn around the wheel's
     *         Z-axis
     */
    public double getDrivingPulsesPerRotation() {
        return this.drivingPulsesPerRotation;
    }

    /**
     * @return The natural rotation vector of the wheel when rotating the robot
     *         around its Z-axis.
     */
    protected Translation2d getRotateVector() {
        return this.naturalRotateVector;
    }

    /**
     * Calculates the steering angle.
     * 
     * @return Steering angle in radians.
     */
    public double getSteeringAngle() {
        return convertEncoderPulsesToRadians(getSteeringEncoderPulses(), getSteeringPulsesPerRotation());
    }

    public Translation2d getMountingPoint() {
        return this.mountingPoint;
    }

    /**
     * Calculates the driving angle.
     * 
     * @return Drive angle in radians.
     */
    public double getDriveAngle() {
        return convertEncoderPulsesToRadians(getDriveEncoderPulses(), getDrivingPulsesPerRotation());
    }

    /**
     * @return Driving speed in meters per second.
     */
    public double getDriveSpeedVelocity() {
        return this.driveSpeed;
    }

    /**
     * Returns the calculated steering position in encoder pulses. Can be used to
     * feed the PID with position control.
     * 
     * @return Steering position in absolute encoder pulses.
     */
    public int getSteeringPosition() {
        return this.steeringPosition;
    }

    /**
     * Get the relative position of the steering wheel in encoder pulses
     * 
     * @return relative position of the steering wheel in encoder pulses
     */
    public abstract double getSteeringEncoderPulses();

    /**
     * Get the relative position of the drive wheel in encoder pulses
     * 
     * @return relative position of the drive wheel in encoder pulses
     */
    public abstract double getDriveEncoderPulses();

    protected double convertEncoderPulsesToRadians(double pulses, double pulsesPerRotation) {
        return (2 * Math.PI) / pulsesPerRotation * pulses;
    }

    protected int convertRadiansToEncoderPulses(double radians, double pulsesPerRotation) {
        return (int) (pulsesPerRotation / (2 * Math.PI) * radians);
    }

    public static double driveMetersPerSecond_to_EncoderTicksPerSecond(double velocity) {
        return (velocity / RobotMap.WHEEL_CIRCUMFERENCE) * RobotMap.SWERVE_DRIVE_ROTATION_ENCODER_TICK_COUNT;
    }

    /*************************** Sendable part begin *****************************/

    @Override
    public void initSendable(SendableBuilder builder) {
        System.out.println("SwerveModule InitSendable called");
        builder.setSmartDashboardType("SwerveModule");
        builder.setSafeState(this::stopMotors);
        builder.addDoubleProperty("Steering Angle", () -> Math.toDegrees(this.getSteeringAngle()), null);
        builder.addDoubleProperty("Steering Angle Goal",
                () -> Math.toDegrees(
                        this.convertEncoderPulsesToRadians(this.getSteeringPosition(), this.steeringPulsesPerRotation)),
                null);
        builder.addDoubleProperty("Drive Speed Goal", this::getDriveSpeedVelocity, null);
        builder.addBooleanProperty("Homing Sensor", this::homingSensorActive, null);
    }

    /*************************** Sendable part ends *****************************/

}
