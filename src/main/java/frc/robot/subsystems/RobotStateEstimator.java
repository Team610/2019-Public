package frc.robot.subsystems;

import java.util.TreeMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.util.Point;

/**
 * RobotStateEstimator
 */
public class RobotStateEstimator extends Subsystem {

    static RobotStateEstimator instance;
    private DriveTrain drive = DriveTrain.getInstance();
    private double leftPrevDist = 0.0;
    private double rightPrevDistance = 0.0;
    private TreeMap<Double, Point> state = new TreeMap<>();
    private boolean zeroed = true;

    private RobotStateEstimator() {
    }

    public synchronized static RobotStateEstimator getInstance() {
        if(instance == null) {
            instance = new RobotStateEstimator();
        }
        return instance;
    }

    public Point getCurrentPose() {
        return state.lastEntry().getValue();
    }

    public void zeroSensors(Point pose) {
        zeroed = true;
        drive.zeroSensors();
        drive.setHeading(pose.getTheta());
        leftPrevDist = drive.getLeftMeters();
        rightPrevDistance = drive.getRightMeters();
        state.clear();
        state.put(Timer.getFPGATimestamp(), new Point(pose.getX(), pose.getY(), pose.getTheta()));
    }

    @Override
    public void zeroSensors() {
        zeroSensors(new Point(0, 0, drive.getHeadingDegrees()));
    }

    @Override
    protected Loop loop() {
        return new Loop(){
        
            @Override
            public void onFirstStart(double time) {
                
            }
        
            @Override
            public void onEnable(double time) {
                zeroSensors();
                leftPrevDist = drive.getLeftMeters();
                rightPrevDistance = drive.getRightMeters();
            }
        
            @Override
            public void onDisable(double time) {
                
            }
        
            @Override
            public void enabledLoop(double time) {
                final double leftDist = drive.getLeftMeters();
                final double rightDist = drive.getRightMeters();
                final double deltaLeftDist = leftDist - leftPrevDist;
                final double deltaRightDist = rightDist - rightPrevDistance;
                final double gyroAngle = drive.getHeadingDegrees();
                final double avgDelta = (deltaLeftDist + deltaRightDist) / 2;
                Point point = new Point(state.lastEntry().getValue().clone());
                point = point.translate(avgDelta, Math.toRadians(gyroAngle));
                point.setTheta(gyroAngle);
                if(!zeroed) {
                    state.put(time, point);
                } else {
                    zeroed = false;
                }
                leftPrevDist = leftDist;
                rightPrevDistance = rightDist;

                SmartDashboard.putString("Pose", getCurrentPose().toString());
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