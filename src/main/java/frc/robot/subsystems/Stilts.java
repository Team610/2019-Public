package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.util.MotorFactory;
import frc.util.Toggle;

public class Stilts extends Subsystem {

    //1 ROTATION OF MOTOR = 2.9268 INCHES UP
    //1 ROTATIONS OF MOTOR = 0.64646464 ROTATION OF WHEELS

    private TalonSRX motor;
    private DoubleSolenoid quad;

    private static Stilts instance = null;
    private StiltState state = StiltState.AUTO;
    private Controls controls;
    private Toggle quadToggle;
    private boolean shouldStiltsLock = true;

    public enum StiltState {
        MANUAL,
        AUTO
    }

    public synchronized static Stilts getInstance() {
        if (instance == null) {
            instance = new Stilts();
        }
        return instance;
    }

    private Stilts() {
        controls = Controls.getInstance();
        quadToggle = new Toggle();
        quadToggle.invertToggleState();
        motor = MotorFactory.createDefaultTalon(Constants.Stilts.MOTOR);
        quad = new DoubleSolenoid(Constants.Stilts.QUAD_FORWARD, Constants.Stilts.QUAD_REVERSE);

        motor.setInverted(true);
        motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100);
        motor.setNeutralMode(NeutralMode.Coast);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        motor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
        motor.overrideLimitSwitchesEnable(false);

        motor.configPeakCurrentLimit(40);
        motor.configContinuousCurrentLimit(30);

        /* Config the peak and nominal outputs, 12V means full */
        motor.configNominalOutputForward(0, 100);
        motor.configNominalOutputReverse(0, 100);
        motor.configPeakOutputForward(1, 100);
        motor.configPeakOutputReverse(-1, 100);

        /**
        * Config the allowable closed-loop error, Closed-Loop output will be
        * neutral within this range. See Table in Section 17.2.1 for native
        * units per rotation.
        */
        motor.configAllowableClosedloopError(0, 0, 100);

        /* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
        motor.config_kF(0, 0, 100);
        motor.config_kP(0, 0, 100);
        motor.config_kI(0, 0, 100);
        motor.config_kD(0, 0, 100);

        
        motor.configClosedLoopPeakOutput(0, 1);

        motor.config_kP(0, 0.15);
    }


    public StiltState getState() {
        return state;
    }

    public void setState(StiltState state) {
        if(!state.equals(this.state)) {
            this.state = state;
        }
    }

    public void setQuad(boolean locked) {
        shouldStiltsLock = locked;
    }

    @Override
    public void zeroSensors() {
        resetTicks();
    } 

    public double getRawTicks() {
        return motor.getSelectedSensorPosition(); //Should be 4096 ticks per rotation
    }

    public ErrorCode getLastError() {
        ErrorCode code = motor.getLastError();
        if(code != ErrorCode.OK) {
        }
        return motor.getLastError();
    }
    
    /** 
     * @return inches of extension from default position
     */
    public double getExtension() {
        return (motor.getSelectedSensorPosition() / 4096) * 2.9268;
    }

    public void resetTicks() {
        motor.setSelectedSensorPosition(0);
    }
    
    public void setTickPosition(double position) {
        motor.set(ControlMode.Position, position);
    }

    public void set(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    protected Loop loop() {
        return new Loop(){
        
            @Override
            public void onFirstStart(double time) {
                
            }
        
            @Override
            public void onEnable(double time) {
                motor.setNeutralMode(NeutralMode.Brake);
                resetTicks();
            }
        
            @Override
            public void onDisable(double time) {
                motor.setNeutralMode(NeutralMode.Coast);
            }
        
            @Override
            public void enabledLoop(double time) {
                SmartDashboard.putNumber("STILTS TICK", getRawTicks());
                switch (state) {
                    case AUTO:
                        break;
                    case MANUAL:  
                        double power = controls.stiltsSpeed();
                        if(quadToggle.getToggleState()) {
                            power *= -1;
                        }
                        motor.set(ControlMode.PercentOutput, power);
                        quadToggle.update(controls.shouldStiltsLock());

                        setQuad(quadToggle.getToggleState());
                    break;
                }
                quad.set(shouldStiltsLock ? Value.kForward : Value.kReverse);
            }
        
            @Override
            public void disabledLoop(double time) {
                
            }
        
            @Override
            public void allLoop(double time) {
                
            }
        };
    }
    
}