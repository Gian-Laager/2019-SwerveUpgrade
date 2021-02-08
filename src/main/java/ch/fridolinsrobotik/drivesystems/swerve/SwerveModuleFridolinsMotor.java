/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import java.util.StringJoiner;

import ch.fridolinsrobotik.motorcontrollers.IFridolinsMotors;
import edu.wpi.first.wpilibj.drive.Vector2d;
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
            int steeringPulsesPerRotation, IFridolinsMotors drivingMotor, int drivingPulsesPerRotation) {
        super(new Translation2d(mountingPosition.x, mountingPosition.y), steeringPulsesPerRotation, drivingPulsesPerRotation);
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

    double maxVel = 0.0;
    @Override
    public void executeSwerveMovement() {
        double driveSpeed = getDriveSpeedVelocity() * SwerveDrive.maxSpeed45PercentOutput;
//         limitRotationOutput(driveSpeed);
        System.out.println("Velocity: " + driveSpeed);
        steeringMotor.setPosition(getSteeringPosition());
        drivingMotor.setVelocity(driveSpeed);
        int mSpeed = drivingMotor.getEncoderVelocity();
        if (mSpeed > maxVel)
            maxVel = mSpeed;
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
    
    @Override
    protected void limitRotationOutput(double velocity) {
        steeringMotor.limitOutput(getLimitedRoationOutput(velocity));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Drive Ticks", () -> this.drivingMotor.getEncoderTicks(), null);
        builder.addDoubleProperty("Drive Speed", () -> this.drivingMotor.getEncoderVelocity(), null); 
        builder.addDoubleProperty("Drive Speed Goal", () -> this.getDriveSpeedVelocity() * SwerveDrive.maxSpeed45PercentOutput, null);
        builder.addDoubleProperty("Drive max speed", () -> this.maxVel, null);
    }
}
