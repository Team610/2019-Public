package frc.robot.auto.actions;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.auto.modes.HangLvl3;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Stilts;
import frc.util.DriveSignal;


/**
 * HangDriveAction
 */

public class HangDriveAction implements Action {

    private DriveTrain driveTrain;
    private Stilts stilts;
    private double driveSpeed, stiltSpeed;
    private boolean isFinished;
    private Arm arm;
    private final double revolutions = 26;

    public HangDriveAction(double driveSpeed, double stiltSpeed) {
        this.driveSpeed = driveSpeed;
        this.stiltSpeed = stiltSpeed;
        arm = Arm.getInstance();
    }

    @Override
    public void start() {
        driveTrain = DriveTrain.getInstance();
        stilts = Stilts.getInstance();
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
    }

    @Override
    public void update() {
        SmartDashboard.putString("Action", "Hang Drive Action");
        SmartDashboard.putNumber("Stilts Tick Left", (stilts.getRawTicks() / 4096) * 0.6464);
        SmartDashboard.putNumber("Stilts Tick Right", revolutions / (4 * Math.PI));
        HangLvl3.errored |= ErrorCode.OK != stilts.getLastError();
        if (arm.getAngle() > Constants.Arm.MAX_FORWARD_SAFE_ANGLE){
            arm.setRotationMotor(0.5);
        } else {
            arm.setRotationMotor(0);
        }
        if ((stilts.getRawTicks() / 4096) * 0.6464 <= revolutions / (4 * Math.PI)) { 
            stilts.setQuad(true);
            stilts.set(stiltSpeed);
            driveTrain.setSpeed(new DriveSignal(driveSpeed, driveSpeed));
        }
        else {
            isFinished = true;
        }

        SmartDashboard.putNumber("Stilt Wheel Ticks", stilts.getRawTicks());
    }


    @Override
    public void done() {
        driveTrain.setSpeed(new DriveSignal(0, 0));
        stilts.set(0);
        stilts.setQuad(true);
        arm.setRotationMotor(0);
    }

    @Override
    public boolean isFinished() {
       return isFinished || HangLvl3.errored;
    }
    
}