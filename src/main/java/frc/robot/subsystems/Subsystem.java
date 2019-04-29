package frc.robot.subsystems;

import frc.loops.Loop;
import frc.loops.LoopManager;

/**
 * ISubsystem
 */
public abstract class Subsystem {

    public Subsystem() {
        SubsystemManager.getInstance().addSubsystem(this);
        LoopManager.getLooper().addLoop(loop());
    }

    /**
     * All sensors or variables that need to be zeroed should be placed here
     */
    public abstract void zeroSensors();

    /**
     * Automatically called and run once the Subsystem object is created in memeory.
     * This handles the enabled, disabled and all periodic loops
     * @return
     */
    protected abstract Loop loop();

}