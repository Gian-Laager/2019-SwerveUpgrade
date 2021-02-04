package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
    private static final int timeoutMs = 20;
    private static final int slotIndex = 0;

    public static class PIDConstants {
        public double kP;
        public double kI;
        public double kD;

        public int tolerance;

        public double acceleration;
        public double cruiseVelocity;
    }

    private static class Motors {
        public static final int encoderTicksPerRotation = 0; //TODO: find out this value
        public WPI_TalonSRX speed;
        public WPI_TalonSRX rotation;
    }
    
    private static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            return new SwerveModuleState(-desiredState.speedMetersPerSecond,
                    desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
        } else {
            return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
        }
    }

    private Motors motors;
    private final Translation2d location;

    public SwerveModule(int motorRotationId, int motorSpeedId, Translation2d location, PIDConstants pidConstants) {
        motors = new Motors();
        motors.speed = new WPI_TalonSRX(motorSpeedId);
        motors.rotation = new WPI_TalonSRX(motorRotationId);
        this.location = location;

        motors.rotation.config_kP(slotIndex, pidConstants.kP, timeoutMs);
        motors.rotation.config_kI(slotIndex, pidConstants.kI, timeoutMs);
        motors.rotation.config_kD(slotIndex, pidConstants.kD, timeoutMs);
        motors.rotation.configAllowableClosedloopError(slotIndex, pidConstants.tolerance, timeoutMs);
        motors.rotation.configMotionAcceleration(pidConstants.acceleration, timeoutMs);
        motors.rotation.configMotionCruiseVelocity(pidConstants.cruiseVelocity, timeoutMs);
        // TODO: change FeedbackDevice to right encoder type
        motors.rotation.configSelectedFeedbackSensor(FeedbackDevice.None /* needs to be changed */ );
    } 

    private double map(double val, double inMin, double inMax, double outMin, double outMax) {
        return outMin + ((outMax - outMin) / (inMax - inMin)) * (val - inMin);
    }

    private double encoderPosToRad(double pos) {
        pos %= Motors.encoderTicksPerRotation;
        return map(pos, 0, Motors.encoderTicksPerRotation, -Math.PI, Math.PI);
    }

    private double rotation2dToEncoderTicks(Rotation2d rotatoin) {
        return map(rotatoin.getRadians(), -Math.PI, Math.PI, 0, Motors.encoderTicksPerRotation);
    }

    public void setDesierdState(SwerveModuleState state) {
        state = optimize(state, new Rotation2d(encoderPosToRad(motors.rotation.getSelectedSensorPosition())));
        motors.rotation.set(ControlMode.MotionMagic, rotation2dToEncoderTicks(state.angle)); 
        motors.speed.set(state.speedMetersPerSecond);
    }

    public void stopMotors() {
        motors.rotation.stopMotor();
        motors.speed.stopMotor();
    }

    public void setSpeedMotorInverted(boolean inverted) {
        motors.speed.setInverted(inverted);
    }

    public void setRotationMotorInverted(boolean inverted) {
        motors.rotation.setInverted(inverted);
    }

    public void setMotorsInverted(boolean rotationInverted, boolean speedInverted) {
        setSpeedMotorInverted(speedInverted);
        setRotationMotorInverted(rotationInverted);
    }
}