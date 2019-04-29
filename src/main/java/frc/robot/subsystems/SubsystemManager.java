package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.loops.Loop;
import frc.loops.LoopManager;
import frc.robot.Constants;
import frc.robot.Controls;
import frc.robot.subsystems.Arm.ArmState;
import frc.robot.subsystems.DriveTrain.DriveState;
import frc.robot.subsystems.Manipulator.ManipState;
import frc.robot.subsystems.Stilts.StiltState;
import frc.util.Point;
import frc.util.Toggle;

/**
 * The SubsystemManger is responsible for controlling the call of the generic
 * subsystem methods. It is also used to link two or more subsystems together in
 * parallel actions.
 */
public class SubsystemManager {

    private HashMap<String, Subsystem> subsystems;

    private static SubsystemManager instance;
    private Controls controls;

    private Point wantedTotalArmPosition = Constants.Presets.getState("CENTER").getPosition();
    private double wantedManipRotation = 0;
    private double wantedIntakeSpeed = 0;
    private TotalState state = TotalState.AUTO;
    private ArmAndManipulatorState curState = null;
    public Toggle isCargoObject;
    private boolean isFrontPreset;

    public enum TotalState {
        AUTO, MANUAL, HANG, NO_CHANGE
    }

    public static SubsystemManager getInstance() {
        if (instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }

    private SubsystemManager() {
        subsystems = new HashMap<>();
        controls = Controls.getInstance();
        isCargoObject = new Toggle();

        LoopManager.getLooper().addLoop(new Loop(){
        
            @Override
            public void onFirstStart(double time) {
                
            }
        
            @Override
            public void onEnable(double time) {
                
            }
        
            @Override
            public void onDisable(double time) {
                
            }
        
            @Override
            public void enabledLoop(double time) {
                if (DriverStation.getInstance().isFMSAttached() 
                && DriverStation.getInstance().isEnabled() 
                && DriverStation.getInstance().isOperatorControl()
                && DriverStation.getInstance().getMatchTime() <= 15
                && DriverStation.getInstance().getMatchTime() >= 14) {
                    controls.operatorRumble(0.2, 0.1);
                }

                isCargoObject.update(controls.toggleGameObject());
                if (controls.shouldGoAuto()) {
                    setState(TotalState.AUTO);
                } else if (controls.shouldGoManual()) {
                    setState(TotalState.MANUAL);
                }
                switch (state) {
                case AUTO:
                    SmartDashboard.putString("SMS", "AUTO_POS");
                    String flag = "";
                    boolean score = false;
                    if (controls.shouldCenterUp()) {
                        flag += "CEN";
                        score = true;
                    } else if (controls.shouldGoToIntakePosition()) {
                        flag += "INT";
                    } else if (controls.isCargoShip()) {
                        flag += "CRS";
                        score = true;
                    } else if (controls.shouldGoToTopLevelRocket() || controls.shouldGoToMidLevelRocket()
                            || controls.shouldGoToLowScoring()) {
                        flag += "ROC";
                        score = true;
                    } else {
                        flag += "NAN";
                    }
                    flag += "_";
                    if (controls.isForwardPreset()) {
                        flag += "FOR";
                    } else {
                        flag += "REV";
                    }
                    flag += "_";
                    if (score) {
                        if (Manipulator.getInstance().hasHatch()) {
                            flag += "HAT";
                        } else {
                            flag += "BAL";
                        }
                    } else if (isCargoObject.getToggleState()) {
                        flag += "HAT";
                    } else {
                        flag += "BAL";
                    }
                    flag += "_";
                    if (controls.shouldGoToTopLevelRocket()) {
                        flag += "TOP";
                    } else if (controls.shouldGoToMidLevelRocket()) {
                        flag += "MID";
                    } else if (controls.shouldGoToLowScoring()) {
                        flag += "LOW";
                    } else {
                        flag += "NAN";
                    }

                    if (flag.contains("INT") && flag.contains("BAL")) {
                        Manipulator.getInstance().setToggleState(false);
                    }

                    SmartDashboard.putString("Flag", flag);
                    ArmAndManipulatorState state = Constants.Presets.getState(flag);
                    if (state != null) {
                        curState = state;
                        isFrontPreset = controls.isForwardPreset();
                        SmartDashboard.putString("Chosen Flag", flag);
                        setWantedTotalArmPosition(curState.getPosition());
                        setWantedManipRotation(curState.getAngle());
                        Manipulator.getInstance().setState(curState.getState());
                        Stilts.getInstance().setState(StiltState.AUTO);
                        Arm.getInstance().setState(ArmState.POSITION_LOCK);
                        DriveTrain.getInstance().setState(DriveState.OPEN_LOOP);
                    } else if (curState == null) {
                        Arm.getInstance().setState(ArmState.MANUAL);
                        Manipulator.getInstance().setState(ManipState.MANUAL);
                        Stilts.getInstance().setState(StiltState.MANUAL);
                        DriveTrain.getInstance().setState(DriveState.OPEN_LOOP);
                    }
                    break;
                case MANUAL:
                    SmartDashboard.putString("SMS", "MANUAL");
                    Arm.getInstance().setState(ArmState.MANUAL);
                    Manipulator.getInstance().setState(ManipState.MANUAL);
                    Stilts.getInstance().setState(StiltState.MANUAL);
                    DriveTrain.getInstance().setState(DriveState.OPEN_LOOP);
                    break;
                case HANG:
                    SmartDashboard.putString("SMS", "HANG");
                    Arm.getInstance().setState(ArmState.HANG);
                    Stilts.getInstance().setState(StiltState.AUTO);
                    DriveTrain.getInstance().setState(DriveState.AUTO_HANG);
                    Manipulator.getInstance().setState(ManipState.OFF);
                    Manipulator.getInstance().setRotation(-0.4);
                    break;
                case NO_CHANGE:
                    break;
                }
                SmartDashboard.putString("SMS Enum", state.name());
                setWantedIntakeSpeed(controls.getIntakePower());
            }
        
            @Override
            public void disabledLoop(double time) {
                
            }
        
            @Override
            public void allLoop(double time) {
                Controls.getInstance().updateRumble();
            }
        });
    }

    public boolean getIsFrontPreset() {
        return isFrontPreset;
    }

    public TotalState getState() {
        return state;
    }

    public void setState(TotalState state) {
        if (!state.equals(this.state)) {
            this.state = state;
        }
    }

    // Will not add duplicate subsystems by matching the class name
    public void addSubsystem(Subsystem subsystem) {
        if (!subsystems.containsKey(subsystem.getClass().getName())) {
            subsystems.put(subsystem.getClass().getName(), subsystem);
        }
    }

    public HashMap<String, Subsystem> getSubsystems() {
        return subsystems;
    }

    // Calls every initated subsystems zeroSensors method
    public void zeroSensors() {
        subsystems.forEach((k, v) -> v.zeroSensors());
    }

    public double getWantedManipRotation() {
        return this.wantedManipRotation;
    }

    public void setWantedManipRotation(double wantedManipRotation) {
        this.wantedManipRotation = wantedManipRotation;
    }

    public double getWantedIntakeSpeed() {
        return this.wantedIntakeSpeed;
    }

    public void setWantedIntakeSpeed(double wantedIntakeSpeed) {
        this.wantedIntakeSpeed = wantedIntakeSpeed;
    }

    /**
     * 
     * @param wantedPosition the x offset from the pivot of the arm. The y offset
     *                       from the pivot of the arm. The angle created from the
     *                       ground plane intersecting with the pivot point of the
     *                       arm.
     */
    public void setWantedTotalArmPosition(Point wantedPosition) {
        this.wantedTotalArmPosition = wantedPosition;
    }

    public Point getWantedTotalArmPosition() {
        return wantedTotalArmPosition;
    }

    /**
     * 
     * @param angle                 in radians
     * @param maxEntensionFromDrive in meters, the max distance extended past drive
     *                              (rules limit 30 inches)
     * @return the maximum distance the arm can extend (in meters) if the arm were
     *         to be in the worst possible position to limit extension. Can return
     *         infinity
     */
    public double getMaxExtensionAtAngle(double angle, double maxExtensionFromDrive) {
        double maxX = maxExtensionFromDrive + Constants.Arm.DIST_TO_FRONT - Constants.Manipulator.LENGTH;
        return maxX / Math.cos(angle);
    }

    public double getGlobalWristAngle() {
        // double globalAngle = 0;
        // if (curState == null) {
        // // If we dont have a last state assume center position
        // globalAngle = Math.PI / 2;
        // } else if (curState.getState() == ManipState.AUTO || curState.getState() ==
        // ManipState.AUTO_GUESS) {
        // // These states want to be perpendicular to the floor at the end
        // globalAngle = getWantedTotalArmPosition().getX() > 0 ? 0 : Math.PI;
        // } else {
        // globalAngle = getWantedManipRotation();
        // }
        // return globalAngle;
        return getWantedManipRotation();
    }

    public boolean shouldManipCenter() {
        return false;//shouldManipCenter;
    }

    public Point getArmPoint() {
        // double globalAngle = getGlobalWristAngle();

        // double xOffset = Math.cos(globalAngle) * Constants.Manipulator.LENGTH;
        // double yOffset = Math.sin(globalAngle) * Constants.Manipulator.LENGTH;

        double newY = getWantedTotalArmPosition().getY(); // - yOffset;
        double newX = getWantedTotalArmPosition().getX();// - xOffset;

        return new Point(newX, newY);
    }

    public double getDesiredArmAngle() {
        Point p = getArmPoint();
        double angle = Math.atan2(p.getY(), Math.abs(p.getX()));
        if (p.getX() < 0) {
            angle = Math.PI - angle;
        }
        return angle;
    }

    public double getDesiredArmExtension() {
        return getArmPoint().hypot();
    }

    public double getDesiredManipAngle() {
        return getGlobalWristAngle();
        // return getDesiredArmAngle()-getGlobalWristAngle();
    }

    /**
     * Only to be used for tests and auto
     */
    public void setCurState(ArmAndManipulatorState state) {
        curState = state;
        if (curState != null) {
            setWantedTotalArmPosition(curState.getPosition());
            setWantedManipRotation(curState.getAngle());
            Manipulator.getInstance().setState(curState.getState());
            Arm.getInstance().setState(ArmState.POSITION_LOCK);
        }
    }
}