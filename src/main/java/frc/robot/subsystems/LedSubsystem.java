package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {

    private static LedSubsystem INSTANCE;

    private DigitalOutput testLed;

    private LEDStatusMode currentStatusMode;
    
    private boolean disablePeriodicEval = false;


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
        /* 
        5 sec before shift starts/ flash tell 2 or 1 second left then go white when pre shoot
        5 seconds before shift ends/ c
        change color when hub lock active /green
        15, 10, 5, 4, 3, 2, 1 tell match end /flash allience color
        alliance color constant / red or blue
        team blue or rainbow when disabled
        */

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
            currentStatusMode = LEDStatusMode.INTAKE;
            int code = currentStatusMode.code;


            testLed.set((code & 1) > 0);
        }
    }

    
    public void setLedStatusMode(LEDStatusMode selectedMode) {
        if (!disablePeriodicEval) {
            currentStatusMode = selectedMode;
        }
    }

    public void clearLedStatus() {
        if (!disablePeriodicEval) {
            currentStatusMode = LEDStatusMode.OFF;
        }
    }

    public void disableLEDs() {
        disablePeriodicEval = true;
    }
    

    public void enableLEDs() {
         disablePeriodicEval = false;
    }

}
