package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.Constants.Timing;
import frc.robot.Constants.Units;
import frc.util.MotorFactory;
import frc.util.SinglePress;
import frc.util.Util;

/**
 * Arm
 */
public class Arm extends Subsystem {

    private static Arm instance;

    private TalonSRX rotationMasterMotor, rotationSlaveMotor;
    private TalonSRX extensionMotor;
    private DoubleSolenoid bikeBrake;

    private ArmState state = ArmState.OFF;
    private SubsystemManager sm;
    private Controls controls;
    private SinglePress trimBack, trimForward;

    private double trim;
    private SinglePress shouldAngleLock;
    private final double smallestSafeExt = Constants.Arm.SHORTEST_LENGTH + 6 * Units.METERS_PER_INCH;
    private double angleToLockTo;
    private boolean constantVoltageDown = false;

    public enum ArmState {
        OFF, MANUAL, POSITION_LOCK, HANG
    }

    public synchronized static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    private Arm() {
        trimBack = new SinglePress();
        trimForward = new SinglePress();
        shouldAngleLock = new SinglePress();
        rotationMasterMotor = MotorFactory.createDefaultTalon(Constants.Arm.ARM_MASTER);
        rotationSlaveMotor = MotorFactory.createSlavedTalon(Constants.Arm.ARM_SLAVE, Constants.Arm.ARM_MASTER);

        rotationMasterMotor.setInverted(true);
        rotationSlaveMotor.setInverted(true);
        rotationMasterMotor.configFeedbackNotContinuous(true, Timing.CAN_TIMEOUT_MS);
        rotationSlaveMotor.configFeedbackNotContinuous(true, Timing.CAN_TIMEOUT_MS);

        extensionMotor = MotorFactory.createDefaultTalon(Constants.Arm.EXTENSION);
        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        extensionMotor.setSensorPhase(true);
        extensionMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen, Constants.Timing.CAN_TIMEOUT_MS);
        extensionMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
                LimitSwitchNormal.NormallyOpen, Constants.Timing.CAN_TIMEOUT_MS);
        extensionMotor.configForwardSoftLimitThreshold(Constants.Arm.FORWARD_SOFT_LIMIT);
        extensionMotor.setControlFramePeriod(ControlFrame.Control_3_General, Constants.Timing.CAN_TIMEOUT_MS);
        extensionMotor.configClearPositionOnLimitR(true, Constants.Timing.CAN_TIMEOUT_MS);
        extensionMotor.configForwardSoftLimitEnable(true);
        extensionMotor.configMotionCruiseVelocity(13_000);
        extensionMotor.configMotionAcceleration(18_000);
        extensionMotor.config_kF(0, 1023.0 / 10_000);
        extensionMotor.config_kP(0, 0.2);
        extensionMotor.config_kI(0, 0.0);
        extensionMotor.config_kD(0, 2);

        rotationMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        rotationMasterMotor.config_kF(0, 0);
        rotationMasterMotor.config_kP(0, 17);
        rotationMasterMotor.config_kI(0, 0.0);
        rotationMasterMotor.config_kD(0, 200);
        rotationMasterMotor.config_IntegralZone(0, (int) getTickFromAngle(Constants.Arm.ANGLE_BRAKE_TOLERANCE),
                Timing.CAN_TIMEOUT_MS);
        rotationMasterMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        rotationMasterMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        rotationMasterMotor.overrideLimitSwitchesEnable(false);

        bikeBrake = new DoubleSolenoid(Constants.Arm.BIKE_BRAKE_FORWARD, Constants.Arm.BIKE_BRAKE_REVERSE);

        sm = SubsystemManager.getInstance();
        controls = Controls.getInstance();
    }

    public void setState(ArmState state) {
        if (!state.equals(this.state)) {
            this.state = state;
        }
    }

    public void setBrake(boolean shouldBrake) {
        bikeBrake.set(shouldBrake ? Value.kForward : Value.kReverse);
    }

    public void positionLock() {
        double angle = sm.getDesiredArmAngle();
        double extension = sm.getDesiredArmExtension();
 
        SmartDashboard.putNumber("Extension", getExtension());
        SmartDashboard.putNumber("Extension Tick", extensionMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Desired Extension", extension);
        SmartDashboard.putNumber("Error", getExtension()-extension);
        SmartDashboard.putNumber("Angle Error", getAngle()-angle);
        SmartDashboard.putNumber("amp", extensionMotor.getOutputCurrent());
        setBrake(Util.withinEpsilon(getAngle(), angle, Constants.Arm.ANGLE_BRAKE_TOLERANCE));
        //If the desired extension is outside of acceptable ranges stop positing locking
        if(getExtension() > sm.getMaxExtensionAtAngle(getAngle(), 30 * Constants.Units.METERS_PER_INCH)) {
            extensionMotor.set(ControlMode.MotionMagic, getTickFromExtension(Constants.Arm.SHORTEST_LENGTH));
            rotationMasterMotor.set(ControlMode.Position, getTickFromAngle(getAngle()));
            SmartDashboard.putString("Current Arm Position State", "Out of Position");
            return;
        }

        // Stay inside safe range to rotate and extend at the same time
        if(!Util.withinEpsilon(getAngle(), angle, Constants.Arm.ANGLE_TOLERANCE) && !Util.withinEpsilon(getExtension(), smallestSafeExt, 0.02)) {
            shouldAngleLock.update(true);
            if (shouldAngleLock.getState()) {
                angleToLockTo = getAngle();
            }
            extensionMotor.set(ControlMode.MotionMagic, getTickFromExtension(smallestSafeExt));
            rotationMasterMotor.set(ControlMode.Position, getTickFromAngle(angleToLockTo));
            SmartDashboard.putString("Current Arm Position State", "Contracting to min");
            // AFrameRGBs.getInstance().setColour(1,0,1);
            return;
        }
        shouldAngleLock.update(false);

       if(!Util.withinEpsilon(getAngle(), angle, Constants.Arm.ANGLE_TOLERANCE)) {
            rotationMasterMotor.set(ControlMode.Position, getTickFromAngle(angle));
            extensionMotor.set(ControlMode.MotionMagic, getTickFromExtension(smallestSafeExt));
            SmartDashboard.putString("Current Arm Position State", "Rotating to new angle");
            // AFrameRGBs.getInstance().setColour(0,1,1);
            return; 
       }

       // Is it the center preset - if so run until the position is close to desired then send constant voltage 
       constantVoltageDown = Util.withinEpsilon(getExtension(), Constants.Presets.getState("CEN").getPosition().hypot(), 0.04) || constantVoltageDown;
       constantVoltageDown &= Util.withinEpsilon(sm.getArmPoint().getX(), Constants.Presets.getState("CEN").getPosition().getX(), 0.01);
       constantVoltageDown &= Util.withinEpsilon(sm.getArmPoint().getY(), Constants.Presets.getState("CEN").getPosition().getY(), 0.01);
       constantVoltageDown &= Util.withinEpsilon(sm.getWantedManipRotation(), Constants.Presets.getState("CEN").getAngle(), 2 * Units.RADIANS_PER_DEGREE);

        if(constantVoltageDown) {
            extensionMotor.set(ControlMode.PercentOutput, -0.2);
            SmartDashboard.putString("Current Arm Position State", "Pulling down"); 
            // AFrameRGBs.getInstance().setColour(1,1,0);
        } else {
            extensionMotor.set(ControlMode.MotionMagic, getTickFromExtension(extension));
            SmartDashboard.putString("Current Arm Position State", "Total position lock"); 
        }
        rotationMasterMotor.set(ControlMode.Position, getTickFromAngle(angle));
        if(Util.withinEpsilon(getExtension(), extension, 0.1)) {
            // AFrameRGBs.getInstance().setColour(0,1,0);
        }
    }

    public void setRotationMotor(double power) {
        setRotationMotor(power, false);
    }

    /**
     * Sets the power [-1,1] with range limiting if the angle is out of the safe
     * range the power will be set to 0
     * 
     * @param power
     */
    public void setRotationMotor(double power, boolean override) {
        if (override) {
            rotationMasterMotor.set(ControlMode.PercentOutput, power);
            return;
        }
        if (rotationMasterMotor.getSelectedSensorPosition() < -800
                || rotationMasterMotor.getSelectedSensorPosition() > 10) {
            DriverStation.reportError("Rotation sensor out of valid range (10, -800) | Was: "
                    + rotationMasterMotor.getSelectedSensorPosition(), false);
        }
        if (getAngle() < Constants.Arm.MAX_FORWARD_SAFE_ANGLE && power > 0) {
            rotationMasterMotor.set(ControlMode.PercentOutput, 0);
            return;
        }
        if (getAngle() > Constants.Arm.MAX_REVERSE_SAFE_ANGLE && power < 0) {
            rotationMasterMotor.set(ControlMode.PercentOutput, 0);
            return;
        }
        rotationMasterMotor.set(ControlMode.PercentOutput, power);
    }

    public void setExtensionMotor(double power) {
        extensionMotor.set(ControlMode.PercentOutput, power);
    }

    public double getExtension() {
        return extensionMotor.getSelectedSensorPosition() * Constants.Arm.EXTENSION_TICK_TO_METERS_SLOPE
                + Constants.Arm.EXTENSION_TICK_TO_METERS_INTERCEPT;
    }

    public double getTickFromExtension(double extension) {
        return (extension - Constants.Arm.EXTENSION_TICK_TO_METERS_INTERCEPT)
                / Constants.Arm.EXTENSION_TICK_TO_METERS_SLOPE;
    }

    public double getTickRotation() {
        return rotationMasterMotor.getSelectedSensorPosition();
    }

    public double getAngle() {
        return (rotationMasterMotor.getSelectedSensorPosition() * Constants.Arm.ANGLE_TICK_TO_RAD_SLOPE
                + Constants.Arm.ANGLE_TICK_TO_RAD_INTERCEPT) - trim;
    }

    public double getTickFromAngle(double radians) {
        return (radians - Constants.Arm.ANGLE_TICK_TO_RAD_INTERCEPT) / Constants.Arm.ANGLE_TICK_TO_RAD_SLOPE;
    }

    /**
     * in meters
     */
    public void setMotionMagicPos(double pos) {
        extensionMotor.set(ControlMode.MotionMagic, getTickFromExtension(pos));
    }

    @Override
    public Loop loop() {
        return new Loop(){
        
            @Override
            public void onFirstStart(double time) {
                
            }
        
            @Override
            public void onEnable(double time) {
                extensionMotor.setNeutralMode(NeutralMode.Brake);
                rotationMasterMotor.setNeutralMode(NeutralMode.Brake);
                rotationSlaveMotor.setNeutralMode(NeutralMode.Brake);
            }
        
            @Override
            public void enabledLoop(double time) {
                trimBack.update(controls.shouldTrimBack());
                trimForward.update(controls.shouldTrimForward());
                if (trimBack.getState()) {
                    trim -= 2 * Units.RADIANS_PER_DEGREE;
                }
                if (trimForward.getState()) {
                    trim += 2 * Units.RADIANS_PER_DEGREE;
                }
                switch (state) {
                case OFF:
                    SmartDashboard.putString("ArmS", "OFF");
                    break;
                case MANUAL:
                    setRotationMotor(controls.getArmRotationPower(), controls.overrideArmSoftLimit());
                    setBrake(controls.brake());
                    extensionMotor.set(ControlMode.PercentOutput, controls.getExtensionPower());
                    SmartDashboard.putString("ArmS", "MANUAL");
                    break;
                case POSITION_LOCK:
                    positionLock();
                    SmartDashboard.putString("ArmS", "POSITION_LOCK");
                    break;
                case HANG:
                    extensionMotor.set(ControlMode.MotionMagic,
                            getTickFromExtension(Constants.Arm.SHORTEST_LENGTH + 8.4 * Units.METERS_PER_INCH));
                    break;
                }
            }

            @Override
            public void disabledLoop(double time) {
                
            }

            @Override
            public void allLoop(double time) {
                SmartDashboard.putNumber("Arm Rad", getAngle());
                SmartDashboard.putNumber("Extension Velocity", extensionMotor.getSelectedSensorVelocity());
                SmartDashboard.putNumber("Trim", trim);
                SmartDashboard.putNumber("Arm", rotationMasterMotor.getSelectedSensorPosition());
            }

            @Override
            public void onDisable(double time) {
                extensionMotor.setNeutralMode(NeutralMode.Coast);
                rotationMasterMotor.setNeutralMode(NeutralMode.Coast);
                rotationSlaveMotor.setNeutralMode(NeutralMode.Coast);
            }
        };

    }

    @Override
    public void zeroSensors() {

    }
    
}