package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {

    private static LedSubsystem INSTANCE;

    private DigitalOutput testLed;

    private LEDStatusMode currentStatusMode;
    
    private boolean disablePeriodicEval;


    private LedSubsystem() {
        testLed = new DigitalOutput(Constants.LEDs.testLedChannel);
        currentStatusMode = LEDStatusMode.OFF;
    }

    public static LedSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LedSubsystem();
        } 
        return INSTANCE;
    }


    /*
    *  Whatever LED status you want to have go inside this Enum
    *  Just make a new status below and add it specific code to it
    */
    public static enum LEDStatusMode {
    
        OFF(0),
        INTAKE(1);

        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }
    }

    /*
    *  Nothing is added to the periodic yet as no LEDs have been added to the robot
    */
    @Override
    public void periodic() {
        if (!disablePeriodicEval) {
        
        }
    }

    
}
