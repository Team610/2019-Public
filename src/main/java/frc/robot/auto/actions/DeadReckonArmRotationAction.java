package frc.robot.auto.actions;

import frc.robot.auto.modes.HangLvl3;
import frc.robot.subsystems.AFrameRGBs;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.AFrameRGBs.RGBState;
import frc.robot.subsystems.Arm.ArmState;

public class DeadReckonArmRotationAction implements Action {

    private boolean isFinished;
    private Arm arm;
    private double tick;
    private boolean forward;

    public DeadReckonArmRotationAction(boolean forward, double tick) {
        this.forward = forward;
        this.tick = tick;
    }

    @Override
    public void start() {
        arm = Arm.getInstance();
        AFrameRGBs.getInstance().setColour(0,1,1);
        AFrameRGBs.getInstance().setState(RGBState.BLINK_FAST);
    }

    @Override
    public void update() {
        arm.setState(ArmState.HANG);
        if (forward) {
            if (arm.getTickFromAngle(arm.getAngle()) < tick) {
                arm.setRotationMotor(1);
            } else {
                isFinished = true;
                arm.setRotationMotor(0);
            }
        } else {
            if (arm.getTickFromAngle(arm.getAngle()) > tick) {
                arm.setRotationMotor(-1);
            } else {
                isFinished = true;
                arm.setRotationMotor(0);
            }
        }
    }

    @Override
    public void done() {
        arm.setRotationMotor(0);
    }

    @Override
    public boolean isFinished() {
        return isFinished || HangLvl3.errored;
    }

}