package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.LoopManager;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.subsystems.SubsystemManager.TotalState;
import frc.util.DriveSignal;
import frc.util.Logger;
import frc.util.MotorFactory;
import frc.util.Toggle;
import frc.util.Util;

/**
 * DriveTrain
 */
public class DriveTrain extends Subsystem {

    private static DriveTrain instance;

    private CANSparkMax leftMaster, leftA;
    private CANSparkMax rightMaster, rightA;
    private Compressor c;
    private Controls controllers;
    private DriveSignal lastSignal;
    private PigeonIMU pidgey;
    private TalonSRX pidgeyTalon;
    private Limelight limelight;

    private double leftOffset, rightOffset;
    private DriveState state = DriveState.OPEN_LOOP;
    private Toggle visionIntake;

    public enum DriveState {
        OPEN_LOOP, VISION_AUTO, AUTO_HANG, VISION_ASSIST, OFF
    }

    public synchronized static DriveTrain getInstance() {
        if (instance == null) {
            instance = new DriveTrain();
        }
        return instance;
    }

    private DriveTrain() {
        // Talon initialization
        leftMaster = MotorFactory.createDefaultBrushlessNeo(Constants.DriveTrain.LEFT_MASTER);
        leftA = MotorFactory.createSlavedNeo(Constants.DriveTrain.LEFT_A, leftMaster);
        rightMaster = MotorFactory.createDefaultBrushlessNeo(Constants.DriveTrain.RIGHT_MASTER);
        rightA = MotorFactory.createSlavedNeo(Constants.DriveTrain.RIGHT_A, rightMaster);

        leftA.setInverted(true);
        leftMaster.setInverted(true);

        c = new Compressor();
        startCompressor();
        controllers = Controls.getInstance();
        pidgeyTalon = MotorFactory.createDefaultTalon(5);
        pidgey = new PigeonIMU(pidgeyTalon);
        limelight = Limelight.getInstance();
        visionIntake = new Toggle();
    }

    public void setState(DriveState state) {
        if (!state.equals(this.state)) {
            this.state = state;
        }
    }

    /**
     * Limits the absolute increase of rate of change of velocity
     * 
     * @param signal
     * @return
     */
    public DriveSignal accelerationLimit(double percentPerSecond, double maxPercent, DriveSignal signal) {
        if (signal == null || lastSignal == null) {
            return new DriveSignal(0, 0);
        }
        double rate = percentPerSecond * LoopManager.getLooper().getDt();
        double maxMax = maxPercent;
        double leftMax = Math.abs(lastSignal.getLeft()) + rate;
        leftMax = Math.min(maxMax, leftMax);
        double rightMax = Math.abs(lastSignal.getRight()) + rate;
        rightMax = Math.min(maxMax, rightMax);
        double limitedLeft = signal.getLeft();
        double limitedRight = signal.getRight();
        if (Math.abs(signal.getLeft()) > leftMax || Math.abs(signal.getRight()) > rightMax) {
            double lOverR = Math.abs(signal.getLeft()) / Math.abs(signal.getRight());
            if (lOverR > 1) {
                limitedLeft = leftMax;
                limitedRight = limitedLeft / lOverR;
            } else {
                limitedRight = rightMax;
                limitedLeft = limitedRight * lOverR;
            }
        }
        limitedLeft = Math.abs(limitedLeft) * Math.signum(signal.getLeft());
        limitedRight = Math.abs(limitedRight) * Math.signum(signal.getRight());
        return new DriveSignal(limitedLeft, limitedRight);
    }

    public void setSpeed(DriveSignal signal) {
        if (controllers.noLimit()) {
            lastSignal = signal;
            leftMaster.set(signal.getLeft());
            rightMaster.set(signal.getRight());
        } else {
            DriveSignal limit = new DriveSignal(signal.getLeft() * 0.85, signal.getRight() * 0.85);
            lastSignal = limit;
            leftMaster.set(limit.getLeft());
            rightMaster.set(limit.getRight());
        }
    }

    public void kajDrive(final double forward, final double turn, double threshhold, double power,
            double sinNonLinearity) {
        double x = Util.clamp(turn, -1, 1);
        double y = Util.clamp(forward, -1, 1);
        x = Util.deadband(x, threshhold);
        y = Util.deadband(y, threshhold);

        double xSign = Math.signum(x);
        double ySign = Math.signum(y);

        for (int i = 0; i < power; i++) {
            x *= x;
            y *= y;
        }
        final double denominator = Math.sin(Math.PI / 2.0 * sinNonLinearity);
        x = Math.sin(Math.PI / 2.0 * sinNonLinearity * x) / denominator;
        x = Math.sin(Math.PI / 2.0 * sinNonLinearity * x) / denominator;
        x = Math.sin(Math.PI / 2.0 * sinNonLinearity * x) / denominator;

        x = xSign * Math.abs(x);
        y = ySign * Math.abs(y);

        setSpeed(new DriveSignal(y + x, y - x));
    }

    public void stopCompressor() {
        c.stop();
    }

    public void startCompressor() {
        c.start();
    }

    public void updateVisionFollower() {
        double y = Util.deadband(controllers.getThrottle(), 0.1);
        double val = limelight.getValue();
        val = Util.clamp(val, -0.4, 0.4);

        double ySign = Math.signum(y);

        for (int i = 0; i < 2; i++) {
            y *= y;
        }

        y = ySign * Math.abs(y);

        setSpeed(new DriveSignal(y + val, y - val));
    }

    public void updateVisionFollowerIntake(double dist) {
        double throttle = limelight.getThrottle(dist);
        SmartDashboard.putNumber("throttle", throttle);
        double val = limelight.getValue();
        val = Util.clamp(val, -0.4, 0.4);
        this.setSpeed(new DriveSignal(throttle + val, throttle - val));
    }

    @Override
    protected Loop loop() {
        return new Loop() {

            @Override
            public void onFirstStart(double time) {
                rightMaster.setIdleMode(IdleMode.kCoast);
                rightA.setIdleMode(IdleMode.kCoast);
                leftMaster.setIdleMode(IdleMode.kCoast);
                leftA.setIdleMode(IdleMode.kCoast);
            }

            @Override
            public void onEnable(double time) {
                rightMaster.setIdleMode(IdleMode.kBrake);
                rightA.setIdleMode(IdleMode.kBrake);
                leftMaster.setIdleMode(IdleMode.kBrake);
                leftA.setIdleMode(IdleMode.kBrake);
            }

            @Override
            public void onDisable(double time) {
                rightMaster.setIdleMode(IdleMode.kCoast);
                rightA.setIdleMode(IdleMode.kCoast);
                leftMaster.setIdleMode(IdleMode.kCoast);
                leftA.setIdleMode(IdleMode.kCoast);
            }

            @Override
            public void enabledLoop(double time) {
                // if (visionIntake.getToggleState()) {
                //     AFrameRGBs.getInstance().setColour(10 / 255.0, 119 / 255.0, 8 / 255.0);
                // } else {
                //     AFrameRGBs.getInstance().setColour(254 / 255.0, 255 / 255.0, 32 / 255.0);
                // }
                SmartDashboard.putNumber("Pitch", getPitch());

                if(controllers.shouldVisionAssist()) {
                    visionIntake.setToggleState(false);
                }

                visionIntake.update(controllers.shouldVisionIntake());

                if (controllers.hatch()) {
                    limelight.setVisionModeState(false);
                    limelight.setTrackingMode(false);
                    setState(DriveState.OPEN_LOOP);
                }

                if (SubsystemManager.getInstance().getState() != TotalState.NO_CHANGE) {
                    if (limelight.getTrackingMode() && visionIntake.getToggleState()) {
                        setState(DriveState.VISION_AUTO);
                    } else if (limelight.getTrackingMode()) {
                        setState(DriveState.VISION_ASSIST);
                    } else {
                        setState(DriveState.OPEN_LOOP);
                    }
                }

                switch (state) {
                case OPEN_LOOP:
                    kajDrive(controllers.getThrottle(), controllers.getCurvature(),
                            Constants.Controls.JOYSTICK_THRESHOLD, Constants.Controls.DRIVE_EXPONENTIAL, 0.7);
                    break;
                case VISION_ASSIST:
                    updateVisionFollower();
                    break;
                case VISION_AUTO:
                    updateVisionFollowerIntake(6);
                    // 5.85 - slow number
                    if (limelight.getArea() > 5.7) {
                        Manipulator.getInstance().setToggleState(!Manipulator.getInstance().getToggleState());
                        limelight.setVisionModeState(false);
                        visionIntake.setToggleState(false);
                        limelight.setTrackingMode(false);
                        setState(DriveState.OPEN_LOOP);
                    }
                    break;
                }
                SmartDashboard.putString("drive State", state.name());
            }

            @Override
            public void disabledLoop(double time) {

            }

            @Override
            public void allLoop(double time) {
                SmartDashboard.putNumber("heading", getHeadingDegrees());
            }
        };
    }

    public double getLeftMeters() {
        return (leftMaster.getEncoder().getPosition() * Math.PI * Constants.DriveTrain.WHEEL_DIAMETER_METERS
                / Constants.DriveTrain.MOTOR_TO_WHEEL_RATIO) - leftOffset;
    }

    public double getRightMeters() {
        return (rightMaster.getEncoder().getPosition() * Math.PI * Constants.DriveTrain.WHEEL_DIAMETER_METERS
                / Constants.DriveTrain.MOTOR_TO_WHEEL_RATIO) - rightOffset;
    }

    public double getHeadingDegrees() {
        double[] ypr = new double[3];
        pidgey.getYawPitchRoll(ypr);
        return -ypr[0];
    }

    public double getHeadingRadians() {
        return Math.toRadians(getHeadingDegrees());
    }

    /**
     * Sets degrees
     */
    public void setHeading(double heading) {
        pidgey.setYaw(-heading);
    }

    @Override
    public void zeroSensors() {
        leftOffset = leftMaster.getEncoder().getPosition();
        rightOffset = rightMaster.getEncoder().getPosition();
        setHeading(0);
    }

    public double getPitch() {
        double[] ypr = new double[3];
        pidgey.getYawPitchRoll(ypr);
        return ypr[1];
    }

}