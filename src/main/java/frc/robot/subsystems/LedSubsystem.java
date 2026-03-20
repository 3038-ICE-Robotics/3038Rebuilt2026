package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LedSubsystem extends SubsystemBase {

    private static LedSubsystem INSTANCE;

    private DigitalOutput pin2;
    private DigitalOutput pin3;
    private DigitalOutput pin4;

    private LEDStatusMode currentStatusMode;

    private boolean disablePeriodicEval = false;

    private LedSubsystem() {
        pin2 = new DigitalOutput(2);
        pin3 = new DigitalOutput(3);
        pin4 = new DigitalOutput(4);
        currentStatusMode = LEDStatusMode.MODE_HUB_LOCK;
    }

    public static LedSubsystem getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new LedSubsystem();
        }
        return INSTANCE;
    }

    /*
     * Whatever LED status you want to have go inside this Enum
     * Just make a new status below and add it specific code to it
     */
    public static enum LEDStatusMode {

        MODE_DISABLED(0),
        MODE_BLUE_ENABLED_NEUTRAL(1),
        MODE_RED_ENABLED_NEUTRAL(2),
        HUB_PREPMODE_HUB_PREPARE(3),
        HUB_AMODE_HUB_ACTIVECTIVE(4),
        MODE_HUB_LOCK(5),
        MODE_END_WARNING(6);

        /*
         * 5 sec before shift starts/ flash tell 2 or 1 second left then go white when
         * pre shoot
         * 5 seconds before shift ends/ c
         * change color when hub lock active /green
         * 15, 10, 5, 4, 3, 2, 1 tell match end /flash allience color
         * alliance color constant / red or blue
         * team blue or rainbow when disabled
         */

        private final int code;

        private LEDStatusMode(int code) {
            this.code = code;
        }
    }

    /*
     * Nothing is added to the periodic yet as no LEDs have been added to the robot
     */
    @Override
    public void periodic() {

        if (!disablePeriodicEval) {
            // currentStatusMode = LEDStatusMode.INTAKE;
            int code = currentStatusMode.code;

            pin2.set((code & 1) == 0);
            pin3.set((code & 2) == 0);
            pin4.set((code & 4) == 0);
            SmartDashboard.putString("LEDSTATUS", currentStatusMode.toString());
            SmartDashboard.putBoolean("LED/Pin2", (code & 1) != 0);
            SmartDashboard.putBoolean("LED/Pin3", (code & 2) != 0);
            SmartDashboard.putBoolean("LED/Pin4", (code & 4) != 0);
        }

    }

    public void setLedStatusMode(LEDStatusMode selectedMode) {
        if (!disablePeriodicEval) {
            currentStatusMode = selectedMode;
        }
    }

    public void clearLedStatus() {
        if (!disablePeriodicEval) {
            currentStatusMode = LEDStatusMode.MODE_DISABLED;
        }
    }

    public void disableLEDs() {
        disablePeriodicEval = true;
    }

    public void enableLEDs() {
        disablePeriodicEval = false;
    }

}
