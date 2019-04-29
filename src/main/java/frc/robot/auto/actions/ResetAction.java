package frc.robot.auto.actions;

import frc.robot.subsystems.DriveTrain;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.HangLvl3;
import frc.robot.subsystems.Stilts;
import frc.util.DriveSignal;

public class ResetAction implements Action {

    private Stilts stilts;
    private DriveTrain driveTrain;
    private boolean isFinished = false;
    private double stiltsTick;

    public ResetAction (double stiltsTick) {
        this.stiltsTick = stiltsTick;
    }

    @Override
    public void start() {
        driveTrain = DriveTrain.getInstance();
        stilts = Stilts.getInstance();
        stilts.setQuad(false);
        stilts.resetTicks();
        stilts.set(-0.1);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        stilts.set(0.1);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        stilts.set(-0.1);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        stilts.set(0.1);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        stilts.set(-0.1);
        try {
            Thread.sleep(125);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        stilts.set(0.1);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        stilts.set(-0.1);
        try {
            Thread.sleep(125);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void update() {
        SmartDashboard.putString("Action", "Reset Action");
        HangLvl3.errored |= ErrorCode.OK != stilts.getLastError();
        
        if (Math.abs(stilts.getRawTicks()) < stiltsTick - 20) {
            stilts.set(-0.4);
            stilts.setQuad(false);
        } else {
            stilts.set(0);
            stilts.setQuad(true);
            isFinished = true;
        }
    }

    @Override
    public void done() {
        stilts.set(0);
        stilts.setQuad(true);
        driveTrain.setSpeed(new DriveSignal(0, 0));
    }

    @Override
    public boolean isFinished() {
        return isFinished || HangLvl3.errored;
    }

}