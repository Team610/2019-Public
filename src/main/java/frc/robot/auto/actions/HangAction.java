package frc.robot.auto.actions;

import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.controlsystems.PID;
import frc.robot.Constants;
import frc.robot.auto.modes.HangLvl3;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Stilts;

public class HangAction implements Action {

    private Stilts stilts;
    private Arm arm;
    private PID pid;
    private double roll;
    private double rollOffset;
    private double stiltsTick;

    private boolean isFinished = false;

    public HangAction(double stiltsTick) {
        this.stiltsTick = stiltsTick;
    }

    @Override
    public void start() {
        stilts = Stilts.getInstance();
        arm = Arm.getInstance();
        stilts.resetTicks();
        rollOffset = DriveTrain.getInstance().getPitch();
        pid = new PID(Constants.Arm.PARALLEL_KP, Constants.Arm.PARALLEL_KI, Constants.Arm.PARALLEL_KD, -1.0, 1.0);
        stilts.setQuad(false);
        // Logger.write("Hang", "Stabalizing robot while lifiting stilts to " + stiltsTick);
    }

    @Override
    public void update() {
        roll = DriveTrain.getInstance().getPitch() - rollOffset; //tilting forwards is positive, tilting backwards is negative

        SmartDashboard.putNumber("STILTS", stilts.getRawTicks());
        HangLvl3.errored |= ErrorCode.OK != stilts.getLastError();
        if (stilts.getRawTicks() < stiltsTick) {
            stilts.setQuad(false);
            stilts.set(1);
        } else {
            stilts.set(0);
            stilts.setQuad(true);
        }
        if (arm.getAngle() > Constants.Arm.MAX_FORWARD_SAFE_ANGLE){
            arm.setRotationMotor(pid.getValue(roll, -3), true);
        } else {
            arm.setRotationMotor(0);
        }
        if(stilts.getRawTicks() > stiltsTick && roll < 4){
            stilts.set(0.5);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            stilts.setQuad(true);
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
            arm.setBrake(true);
            arm.setRotationMotor(0);
            stilts.set(0);
            isFinished = true;
        }

    }

    @Override
    public void done() {
        arm.setRotationMotor(0);
        stilts.setQuad(true);
        stilts.set(0);
        stilts.resetTicks();
    }

    @Override
    public boolean isFinished() {
        return isFinished || HangLvl3.errored;
    }

}