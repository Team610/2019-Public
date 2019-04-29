package frc.robot.auto.actions;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.DriveTrain.DriveState;
import frc.util.DriveSignal;

/**
 * IntakeDriveTillDistance
 */
public class VisionDriveTillDistance implements Action {

    private DriveTrain dt;
    private Limelight ll;
    private double driveDist;

    /**
     * 
     * @param distance - in inches
     */
    public VisionDriveTillDistance(double driveDist) {
        this.driveDist = driveDist;
        dt = DriveTrain.getInstance();
        ll = Limelight.getInstance();
    }

    @Override
    public void start() {
        dt.setState(DriveState.OFF);
        ll.setVisionModeState(true);
        ll.setTrackingMode(true);
        System.out.println("Vision mode");
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    @Override
    public void update() {
        dt.updateVisionFollowerIntake(driveDist);
    }

    @Override
    public boolean isFinished() {
        return ll.getArea() > driveDist;
    }

    @Override
    public void done() {
        System.out.println("Done vision drive");
        dt.setSpeed(new DriveSignal(0,0));
        ll.setVisionModeState(false);
        ll.setTrackingMode(false);
    }
    
}