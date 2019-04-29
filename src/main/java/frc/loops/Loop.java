package frc.loops;

/**
 * Loop
 */
public interface Loop {

    /**
     * Called at the end of robot init
     * It is not guarrenteed to be called before any teleop or autonomous init 
     * commands if not enough time is given for the threads to start usually around
     * ~5 seconds
     * @param time
     */
    public void onFirstStart(double time);

    /**
     * Called at the start of every init method that will enable the robot
     * @param time
     */
    public void onEnable(double time);

    /**
     * Loops periodically if the DriverStation object returns that the robot is enabled
     * @param time
     */
    public void enabledLoop(double time);

    /**
     * Loops periodically if the DriverStation object returns that the robot is disabled
     * @param time
     */
    public void disabledLoop(double time);

    /**
     * Loops periodically no matter the state of the robot
     * @param time
     */
    public void allLoop(double time);

    /**
     * Called at the start of every init method that will disable the robot
     * @param time
     */
    public void onDisable(double time);
    
}