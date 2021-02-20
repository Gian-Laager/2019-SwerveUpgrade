/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import java.util.StringJoiner;

import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import ch.fridolinsrobotik.utilities.Pair;
import ch.fridolinsrobotik.utilities.CSVLogger;
import ch.fridolinsrobotik.utilities.Timer;
import ch.fridolinsrobotik.utilities.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SwerveModuleFridolinsMotor extends SwerveModule {
    private static int instances;
    IFridolinsMotors drivingMotor;
    IFridolinsMotors steeringMotor;

    public SwerveModuleFridolinsMotor(Vector2d mountingPosition, IFridolinsMotors steeringMotor,
            double steeringPulsesPerRotation, IFridolinsMotors drivingMotor, double drivingPulsesPerRotation) {
        super(new Translation2d(mountingPosition.x, mountingPosition.y), steeringPulsesPerRotation,
                drivingPulsesPerRotation);
        verify(drivingMotor, steeringMotor);
        this.drivingMotor = drivingMotor;
        this.steeringMotor = steeringMotor;
        super.wheelVector = Vector2d.fromPolar(getDriveAngleFromEncoder().getRadians(), getDriveSpeedVelocity());
        SendableRegistry.setName(this, "SwerveModuleFridolinsMotor", instances);
        SendableRegistry.addChild(this, drivingMotor);
        SendableRegistry.addChild(this, steeringMotor);
        instances++;
//         csvLogger = new CSVLogger("/tmp/" + SendableRegistry.getName(this).replace("[", "_").replace("]", "") + ".csv");
    }

    /**
     * Verifies that all motors are nonnull, throwing a NullPointerException if any
     * of them are. The exception's error message will specify all null motors, e.g.
     * {@code
     * NullPointerException("drivingMotor, steeringMotor")}, to give as much
     * information as possible to the programmer.
     *
     * @throws NullPointerException if any of the given motors are null
     */
    private void verify(IFridolinsMotors drivingMotor, IFridolinsMotors steeringMotor) {
        if (drivingMotor != null && steeringMotor != null) {
            return;
        }
        StringJoiner joiner = new StringJoiner(", ");
        if (drivingMotor == null) {
            joiner.add("drivingMotor");
        }
        if (steeringMotor == null) {
            joiner.add("steeringMotor");
        }
        throw new NullPointerException(joiner.toString());
    }

    @Override
    public void executeSwerveMovement() {
        steeringMotor.setPosition(getSteeringPosition());
        drivingMotor.setVelocity(driveMetersPerSecond_to_EncoderTicksPerSecond(getDriveSpeedVelocity()));
    }

    @Override
    public boolean homingSensorActive() {
        return steeringMotor.isForwardLimitSwitchActive();
    }

    @Override
    public void setSteeringVelocity(int unitsPer100ms) {
        steeringMotor.setVelocity(unitsPer100ms);
    }

    @Override
    public void stopMotors() {
        steeringMotor.setPercent(0);
        drivingMotor.setPercent(0);
    }

    @Override
    public void resetSteeringEncoder() {
        steeringMotor.setPosition(0);
    }

    @Override
    public void resetDrivingEncoder() {
        drivingMotor.setPosition(0);
    }

    @Override
    public double getSteeringEncoderPulses() {
        return steeringMotor.getEncoderTicks();
    }

    @Override
    public double getDriveEncoderPulses() {
        return drivingMotor.getEncoderTicks();
    }

    private Vector2d getBestSolutionOfInverseDotProduct(Pair<Vector2d, Vector2d> solutions, Vector2d moduleRotation_,
            Vector2d actualTargetVector) {
        Vector2d bestSoution;
        Vector2d moduleRotation = moduleRotation_.clone();
        moduleRotation.y *= driveInverted;
        if (solutions.first.dot(actualTargetVector) > solutions.second.dot(actualTargetVector)
                ^ moduleRotation.dot(actualTargetVector) < 0.0)
            bestSoution = solutions.first;
        else
            bestSoution = solutions.second;

        if (Math.abs(moduleRotation.dot(actualTargetVector)) > moduleRotation.dot(bestSoution))
            bestSoution = actualTargetVector;

        return bestSoution;
    }

    /**
     * @param velocity The current velocity of the swerve module
     * @return The time that the limited output would take to make the curve
     */
    private double getDelayOfSmothedCurve(double velocity) {
        return Math.PI / 2 * Math.exp(velocity * velocity * gauseFactor);
    }

    private Timer calcTargetDelay = new Timer();

    private Vector2d calcTargetAfterdelay(Vector2d moduleRotatoin, Vector2d targetRotation, double velocity) {
        Vector2d targetAfterdelay = targetRotation;
        if (calcTargetDelay.getLastStarted() != -1 && lastTargetVector != null) {
            double deltaAngle = Math.acos(lastTargetVector.dot(targetRotation)) * calcTargetDelay.getLastStarted()
                    / 1000;
            deltaAngle *= getDelayOfSmothedCurve(velocity);
            targetAfterdelay = Vector2d.fromRad(deltaAngle);
        }

        calcTargetDelay.start();
        return targetAfterdelay;
    }

    @Override
    protected Vector2d getLimitedSteeringVector(Vector2d moduleRotation_, Vector2d targetRotation, double velocity) {
        // Vector2d targetAfterdelay = calcTargetAfterdelay(moduleRotation,
        // targetRotation, velocity);
        Vector2d moduleRotation = moduleRotation_.clone();
        if (Math.signum(moduleRotation.dot(targetRotation)) < 0.0)
            moduleRotation.rotate(180);
        double velocityPercent = ((velocity / RobotMap.WHEEL_CIRCUMFERENCE)
                * RobotMap.SWERVE_DRIVE_ROTATION_ENCODER_TICK_COUNT) / SwerveDrive.maxSpeed45PercentOutput * 0.45;
//         csvLogger.put("Velocity percent", velocityPercent);
        Pair<Vector2d, Vector2d> limitedTargetVectors = moduleRotation.normalize()
                .inverseDot(getLimitedDotProduct(velocityPercent));
        Vector2d limitedTargetVector = getBestSolutionOfInverseDotProduct(limitedTargetVectors, moduleRotation,
                targetRotation);
        // if (limitedTargetVector.dot(moduleRotation) >
        // targetAfterdelay.dot(moduleRotation))
        // return limitedTargetVector;
        // return targetRotation;
        return limitedTargetVector;
    }

    @Override
    public Rotation2d getDriveAngleFromEncoder() {
        return new Rotation2d(
                (steeringMotor.getEncoderTicks() / RobotMap.SWERVE_STEER_ROTATION_ENCODER_TICK_COUNT) * 2 * Math.PI);
    }

    @Override
    public double getDriveVelocityFromEncoder() {
        return (steeringMotor.getEncoderVelocity() / RobotMap.SWERVE_DRIVE_ROTATION_ENCODER_TICK_COUNT)
                * RobotMap.WHEEL_CIRCUMFERENCE;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Drive Ticks", () -> this.drivingMotor.getEncoderTicks(), null);
        builder.addDoubleProperty("Drive Speed", () -> this.drivingMotor.getEncoderVelocity(), null);
        builder.addDoubleProperty("Drive Speed Goal",
                () -> this.getDriveSpeedVelocity() * SwerveDrive.maxSpeed45PercentOutput, null);
    }
}
