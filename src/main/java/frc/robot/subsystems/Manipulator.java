package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controlsystems.PID;
import frc.loops.Loop;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.util.MotorFactory;
import frc.util.Toggle;

/**
 * Manipulator
 */
public class Manipulator extends Subsystem {

    private static Manipulator instance;

    private PID quadOneAnglePID, quadTwoAnglePID;
    
    private TalonSRX rollerAMotorMaster, rollerBMotor;
    private TalonSRX rotationMotor;
    private DoubleSolenoid hatch;

    private SubsystemManager sm;
    private ManipState state = ManipState.OFF;
    private Controls controls;
    private Toggle hatchToggle;
    private boolean runIntake;

    public enum ManipState {
        OFF,
        MANUAL,
        ANGLE_LOCK,
        AUTO,
        AUTO_GUESS
    }

    public synchronized static Manipulator getInstance() {
        if(instance == null) {
            instance = new Manipulator();
        }
        return instance;
    }

    private Manipulator() {
        sm = SubsystemManager.getInstance();
        hatch = new DoubleSolenoid(Constants.Manipulator.HATCH_FORWARD, Constants.Manipulator.HATCH_REVERSE);
        hatchToggle = new Toggle();

        rollerAMotorMaster = MotorFactory.createDefaultTalon(Constants.Manipulator.ROLLER_A);
        rollerBMotor = MotorFactory.createSlavedTalon(Constants.Manipulator.ROLLER_B, Constants.Manipulator.ROLLER_A);
        rotationMotor = MotorFactory.createDefaultTalon(Constants.Manipulator.WRIST);
        
        rollerAMotorMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        rollerAMotorMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        rollerAMotorMaster.overrideLimitSwitchesEnable(false);
        rollerBMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        rollerBMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        rollerBMotor.overrideLimitSwitchesEnable(false);

        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
        rotationMotor.configContinuousCurrentLimit(10);
        rotationMotor.setInverted(false);
        rotationMotor.setNeutralMode(NeutralMode.Brake);
        rotationMotor.enableCurrentLimit(true);

        quadOneAnglePID = new PID(Constants.Manipulator.KP, Constants.Manipulator.KD, Constants.Manipulator.KI, -1, 1);
        quadTwoAnglePID = new PID(-Constants.Manipulator.KP, -Constants.Manipulator.KD, -Constants.Manipulator.KI, -1, 1);
        controls = Controls.getInstance();
    }

    public void setState(ManipState state) {
        if(!state.equals(this.state)) {
            this.state = state;
        }
    }

    /**
     * @param state - false holds hatch | true outtakes hatch
     */
    public void setToggleState(boolean state) {
        hatchToggle.setToggleState(state);
    }

    public boolean getToggleState() {
        return hatchToggle.getToggleState();
    }

    public void setIntake(double power) {
        rollerAMotorMaster.set(ControlMode.PercentOutput, power);
    }

    public void setRotation(double power) {
        if (getTick() < Constants.Manipulator.MAX_FORWARD_VALUE && power > 0) {
            rotationMotor.set(ControlMode.PercentOutput, 0);
            AFrameRGBs.getInstance().setColour(1,1,0);
            return;
        }
        if (getTick() > Constants.Manipulator.MAX_REVERSE_VALUE && power < 0) {
            rotationMotor.set(ControlMode.PercentOutput, 0);
            AFrameRGBs.getInstance().setColour(1,1,0);
            return;
        }
        AFrameRGBs.getInstance().setColour(0,1,0);
        rotationMotor.set(ControlMode.PercentOutput, power);
    }

    public void angleLock(double d) {
        setRotation(quadOneAnglePID.getValue(getAngle(), d));
    }

    public void angleLock() {
        setRotation(quadOneAnglePID.getValue(getAngle(), sm.getDesiredManipAngle()));
    }

    public void scoreLock() {
        double angle = -Arm.getInstance().getAngle();

        double power = quadOneAnglePID.getValue(getAngle(), angle);
        if (-angle > Math.PI/2) {
            angle = -angle - Math.PI;
            power = quadTwoAnglePID.getValue(-getAngle(), angle);
        }

        setRotation(power);
    }

    public void scoreLock(double angleFromArm) {
        double angle = -angleFromArm;

        double power = quadOneAnglePID.getValue(getAngle(), angle);
        if (-angle > Math.PI/2) {
            angle = -angle - Math.PI;
            power = quadTwoAnglePID.getValue(-getAngle(), angle);
        }

        setRotation(power);
    }

    public double getTick() {
        return rotationMotor.getSelectedSensorPosition();
    }

    public double getAngle() {
        return rotationMotor.getSelectedSensorPosition() * Constants.Manipulator.ANGLE_TICK_TO_RAD_SLOPE + Constants.Manipulator.ANGLE_TICK_TO_RAD_INTERCEPT;
    }
    
    public boolean hasHatch() {
        return rollerAMotorMaster.getSensorCollection().isFwdLimitSwitchClosed();
    }

    @Override
    protected Loop loop() {
        return new Loop(){
        
            @Override
            public void onFirstStart(double time) {
                
            }
        
            @Override
            public void onEnable(double time) {
                rollerAMotorMaster.setNeutralMode(NeutralMode.Brake);
            }
        
            @Override
            public void onDisable(double time) {
                rollerAMotorMaster.setNeutralMode(NeutralMode.Coast);
            }
        
            @Override
            public void enabledLoop(double time) {
                double intakePower = 0;
                if(sm.getWantedIntakeSpeed() > 0 && !rollerAMotorMaster.getSensorCollection().isFwdLimitSwitchClosed()) {
                    runIntake = true;
                }
                if(sm.getWantedIntakeSpeed() < 0) {
                    runIntake = false;
                }

                switch (state) {
                    case OFF:
                        break;
                    case MANUAL:
                        if(runIntake) {
                            intakePower = Constants.Manipulator.BALL_STALL_SPEED;
                        }
                        setRotation(controls.getManipRotationPower());
                        break;
                    case ANGLE_LOCK:
                        if(sm.shouldManipCenter()) {
                            angleLock(0);
                        } else {
                            angleLock();
                        }
                        if(runIntake) {
                            intakePower = Constants.Manipulator.BALL_STALL_SPEED;
                        }
                        break;
                    case AUTO:
                        if(sm.shouldManipCenter()) {
                            angleLock(0);
                        } else {
                            scoreLock();
                        }
                        if(runIntake) {
                            intakePower = Constants.Manipulator.BALL_STALL_SPEED;
                        }
                        break;
                    case AUTO_GUESS:
                        if(runIntake) {
                            intakePower = Constants.Manipulator.BALL_STALL_SPEED;
                        }
                        if(sm.shouldManipCenter()) {
                            angleLock(0);
                        } else {
                            scoreLock(sm.getDesiredArmAngle());
                        }
                    break;

                }
                if(sm.getWantedIntakeSpeed() != 0) {
                    intakePower = sm.getWantedIntakeSpeed();
                }
                SmartDashboard.putString("Manip state", state.name());
                hatchToggle.update(Controls.getInstance().hatch());
                setIntake(intakePower);
                hatch.set(hatchToggle.getToggleState() ? Value.kReverse : Value.kForward);
            }
        
            @Override
            public void disabledLoop(double time) {
                
            }
        
            @Override
            public void allLoop(double time) {
                SmartDashboard.putBoolean("toggle", hatchToggle.getToggleState());
                SmartDashboard.putNumber("MANIP Tick", rotationMotor.getSelectedSensorPosition());
                SmartDashboard.putNumber("MANIP", rotationMotor.getSelectedSensorPosition());
                SmartDashboard.putBoolean("HATCH", hasHatch());
                SmartDashboard.putNumber("CURRENT A", rollerAMotorMaster.getOutputCurrent());
                SmartDashboard.putNumber("CURRENT B", rollerBMotor.getOutputCurrent());
            }
        };
    }

    @Override
    public void zeroSensors() {
        
    }
    
}