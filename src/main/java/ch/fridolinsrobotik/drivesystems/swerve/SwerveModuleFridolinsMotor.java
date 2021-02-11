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
import ch.fridolinsrobotik.utilities.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SwerveModuleFridolinsMotor extends SwerveModule {
    private static int instances;
    IFridolinsMotors drivingMotor;
    IFridolinsMotors steeringMotor;

    public SwerveModuleFridolinsMotor(Vector2d mountingPosition, IFridolinsMotors steeringMotor,
            int steeringPulsesPerRotation, IFridolinsMotors drivingMotor, int drivingPulsesPerRotation) {
        super(new Translation2d(mountingPosition.x, mountingPosition.y), steeringPulsesPerRotation,
                drivingPulsesPerRotation);
        verify(drivingMotor, steeringMotor);
        this.drivingMotor = drivingMotor;
        this.steeringMotor = steeringMotor;
        SendableRegistry.setName(this, "SwerveModuleFridolinsMotor", instances);
        SendableRegistry.addChild(this, drivingMotor);
        SendableRegistry.addChild(this, steeringMotor);
        instances++;
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

    private Vector2d getBestSolutionOfInverseDotProduct(Pair<Vector2d, Vector2d> solutions, Vector2d targetVector) {
        if (solutions.first.dot(targetVector) > solutions.second.dot(targetVector))
            return solutions.first;
        else 
            return solutions.second;
    }

    @Override
    protected Vector2d getLimitedSteeringVector(Vector2d moduleRotation, Vector2d targetRotation) {
        double velocityPercent = (Robot.swerve.getRobotVelocity().magnitude() / SwerveDrive.maxSpeed45PercentOutput) * (1 / 0.45);
        Pair<Vector2d, Vector2d> limitedTargetVectors = moduleRotation.normalize().inverseDot(getLimitedDotProduct(velocityPercent));
        return getBestSolutionOfInverseDotProduct(limitedTargetVectors, targetRotation);
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
