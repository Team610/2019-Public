package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SubsystemManager;
import frc.util.Util;

public class SetArmFlagAction implements Action {

    private String flag;
    private SubsystemManager sm;

    public SetArmFlagAction(String flag) {
        this.flag = flag;
    }

    @Override
    public void start() {
        sm = SubsystemManager.getInstance();
        sm.setCurState(Constants.Presets.getState(flag));
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        System.out.println("Done going to FLAG preset");
    }

    @Override
    public boolean isFinished() {
        boolean done = true;
        done &= Util.withinEpsilon(Arm.getInstance().getExtension(), sm.getDesiredArmExtension(), 0.05);
        done &= Util.withinEpsilon(Arm.getInstance().getAngle(), sm.getDesiredArmAngle(), 0.05);
        return done;
    }

}