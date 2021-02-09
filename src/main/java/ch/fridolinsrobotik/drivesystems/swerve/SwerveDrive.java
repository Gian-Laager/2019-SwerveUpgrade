/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package ch.fridolinsrobotik.drivesystems.swerve;

import java.util.Arrays;

import ch.fridolinsrobotik.utilities.Algorithms;
import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * A class for driving Swerve drive platforms.
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
 * <p>
 * Inputs smaller then
 * {@value ch.fridolinsrobotik.drivesystems.swerve.SwerveDrive#kDefaultDeadband}
 * will be set to 0, and larger values will be scaled so that the full range is
 * still used. This deadband value can be changed with {@link #setDeadband}.
 * </p>
 */
public class SwerveDrive extends MotorSafety implements Sendable {
    SwerveDriveKinematics kinematics;
    SwerveModule[] swerveModules;

    public static final double kDefaultDeadband = 0.04;
    public static final double kDefaultSpeedFactor = 1.0;

    protected double m_deadband = kDefaultDeadband;
    protected double m_speedFactor = kDefaultSpeedFactor;

    private static int instances;

    public SwerveDrive() {
        // m_sendableImpl = new SendableImpl(true);
        super();
        String name = getClass().getName();
        name = name.substring(name.lastIndexOf('.') + 1);
        instances++;
        setSafetyEnabled(true);
        SendableRegistry.addLW(this, name, name);
        SendableRegistry.setName(this, "SwerveDrive", instances);
        SendableRegistry.enableLiveWindow(this);
    }

    public void addSwerveModules(SwerveModule... modules) {
        this.swerveModules = modules.clone();
        Translation2d[] mountingPoints = Arrays.stream(modules).map(SwerveModule::getMountingPoint)
                .toArray(Translation2d[]::new);
        kinematics = new SwerveDriveKinematics(mountingPoints);
        // for (SwerveModule module : this.swerveModules)
        // SendableRegistry.addChild(this, module);
    }

    // public ArrayList<SwerveModule> getSwerveModules() {
    // return this.swerveModules;
    // }

    public double getDeadband() {
        return this.m_deadband;
    }

    public void setDeadband(double m_deadband) {
        this.m_deadband = m_deadband;
    }

    public double getSpeedFactor() {
        return this.m_speedFactor;
    }

    public void setSpeedFactor(double speedFactor) {
        this.m_speedFactor = speedFactor;
    }

    /**
     * Drive method for Mecanum platform.
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
    public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
        driveCartesian(ySpeed, xSpeed, zRotation, 0.0);
    }

    private Vector2d swerveModuleStateToVector2d(SwerveModuleState state) {
        Vector2d retVector = new Vector2d();
        retVector.x = Math.cos(state.angle.getRadians()) * state.speedMetersPerSecond;
        retVector.y = Math.sin(state.angle.getRadians()) * state.speedMetersPerSecond;
        return retVector;
    }

    private SwerveDriveKinematics middleWheelSwervedrive = new SwerveDriveKinematics(new Translation2d(0.0, 0.0),
            new Translation2d(0.0, 0.0));
    private Vector2d robotVelocity;

    /**
     * @return Velocity vector of the whole robot. Length is the speed that the
     *         robot drives in the direction of the vector in m/s
     */
    public Vector2d getRobotVelocity() {
        return robotVelocity;
    }

    /**
     * Drive method for Mecanum platform.
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
     * @param gyroAngle The current angle reading from the gyro in degrees around
     *                  the Z axis. Use this to implement field-oriented controls.
     */
    public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
        ySpeed = MathUtil.clamp(ySpeed, -1, 1);
        xSpeed = MathUtil.clamp(xSpeed, -1, 1);
        zRotation = MathUtil.clamp(zRotation, -1, 1);

        /*
         * apply deadband to rotation to prevent jittering of the steering.
         */
        if (Math.abs(zRotation) < m_deadband) {
            zRotation = 0;
        } else if (zRotation >= m_deadband) {
            zRotation = Algorithms.scale(zRotation, m_deadband, 1, 0, 1);
        } else {
            zRotation = Algorithms.scale(zRotation, -1, m_deadband, -1, 0);
        }
        // System.out.println(String.format("SwerveDrive: y speed: %f, x speed: %f,
        // zRotation: %f", ySpeed, xSpeed, zRotation));

        Translation2d striveVector = new Translation2d(xSpeed, ySpeed);
        striveVector.rotateBy(Rotation2d.fromDegrees(-gyroAngle));

        /*
         * apply deadband without disturbing the steering angle to prevent abrupt
         * stopping.
         */
        if (striveVector.getNorm() >= m_deadband) {
            striveVector = Algorithms.scale(striveVector, m_deadband, 1, 0, 1);
        } else {
            striveVector = new Translation2d(0.0, 0.0);
        }
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(striveVector.getX(), -striveVector.getY(),
                Math.PI * zRotation, Rotation2d.fromDegrees(0/*-gyroAngle*/));
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState robotState = middleWheelSwervedrive.toSwerveModuleStates(speeds)[0];
        robotVelocity = swerveModuleStateToVector2d(robotState);
        for (int i = 0; i < moduleStates.length; ++i) {
            SwerveModule module = this.swerveModules[i];
            SwerveModuleState moduleState = moduleStates[i];
            // System.out.println("I: "+ i + " " + moduleState.toString() + " Strive vector:
            // " + striveVector.toString());

            if (Math.abs(zRotation) > 0 || Math.hypot(striveVector.getX(), striveVector.getY()) > 0) {
                module.calculateSwerveMovement(moduleState.speedMetersPerSecond, moduleState.angle);
                module.setDriveSpeedVelocity(module.getDriveSpeedVelocity() /* * this.m_speedFactor */);
            } else
                module.setDriveSpeedVelocity(0.0);
        }

        normalize();

        for (SwerveModule module : this.swerveModules) {
            module.executeSwerveMovement();
        }
        feed();
    }

    /**
     * Drive method for Swerve platform.
     *
     * <p>
     * Angles are measured counter-clockwise from straight ahead. The speed at which
     * the robot drives (translation) is independent from its angle or rotation
     * rate.
     *
     * @param magnitude The robot's speed at a given angle [-1.0..1.0]. Forward is
     *                  positive.
     * @param angle     The angle around the Z axis at which the robot drives in
     *                  degrees [-180..180].
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0].
     *                  Clockwise is positive.
     */
    public void drivePolar(double magnitude, double angle, double zRotation) {
        driveCartesian(magnitude * Math.sin(angle * (Math.PI / 180.0)), magnitude * Math.cos(angle * (Math.PI / 180.0)),
                zRotation, 0.0);
    }

    /**
     * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     */
    protected static final double maxSpeed45PercentOutput = 14132.0;

    protected void normalize() {
        double maxMagnitude = 0;
        for (int i = 0; i < this.swerveModules.length; i++) {
            double temp = Math.abs(this.swerveModules[i].getDriveSpeedVelocity());
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > maxSpeed45PercentOutput) {
            for (int i = 0; i < this.swerveModules.length; i++) {
                SwerveModule module = this.swerveModules[i];
                module.setDriveSpeedVelocity((module.getDriveSpeedVelocity() / maxMagnitude) * maxSpeed45PercentOutput);
            }
        }
    }

    /************************* MotorSafety part ends ***************************/

    public void stopMotor() {
        for (SwerveModule module : this.swerveModules) {
            module.stopMotors();
        }
        feed();
    }

    @Override
    public String getDescription() {
        return "SwerveDrive";
    }

    /************************** MotorSafety part ends ****************************/

    /*************************** Sendable part begin *****************************/

    // @Override
    // public void close() {
    // m_sendableImpl.close();
    // }

    // @Override
    // public final synchronized String getName() {
    // return m_sendableImpl.getName();
    // }

    // @Override
    // public final synchronized void setName(String name) {
    // m_sendableImpl.setName(name);
    // }

    // /**
    // * Sets the name of the sensor with a channel number.
    // *
    // * @param moduleType A string that defines the module name in the label for
    // the
    // * value
    // * @param channel The channel number the device is plugged into
    // */
    // protected final void setName(String moduleType, int channel) {
    // m_sendableImpl.setName(moduleType, channel);
    // }

    // /**
    // * Sets the name of the sensor with a module and channel number.
    // *
    // * @param moduleType A string that defines the module name in the label for
    // * the value
    // * @param moduleNumber The number of the particular module type
    // * @param channel The channel number the device is plugged into (usually
    // * PWM)
    // */
    // protected final void setName(String moduleType, int moduleNumber, int
    // channel) {
    // m_sendableImpl.setName(moduleType, moduleNumber, channel);
    // }

    // @Override
    // public final synchronized String getSubsystem() {
    // return m_sendableImpl.getSubsystem();
    // }

    // @Override
    // public final synchronized void setSubsystem(String subsystem) {
    // m_sendableImpl.setSubsystem(subsystem);
    // }

    // /**
    // * Add a child component.
    // *
    // * @param child child component
    // */
    // protected final void addChild(Object child) {
    // m_sendableImpl.addChild(child);
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDriveKinematic");
        builder.setSafeState(this::stopMotor);
        builder.addBooleanProperty("Test", () -> true, null);
    }

    /*************************** Sendable part ends *****************************/
}
