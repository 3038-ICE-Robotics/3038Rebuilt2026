package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.StateOfRobot;

/**
*LedSubsystem to control robot's LEDs, by determining what number should be endcoded to DIO pins
*sent to the Arduino board to control patterns and colors.
*/

public class LedSubsystem extends SubsystemBase {
    private static LedSubsystem INSTANCE;

    private final DigitalOutput codeChannel2, codeChannel3, codeChannel4;

    private LEDStatusMode currentStatusMode;

    private boolean disablePeriodicEval = false;

    public static Optional<Alliance> alliance = DriverStation.getAlliance();

    private static double matchTime = DriverStation.getMatchTime();

    private String gameData = DriverStation.getGameSpecificMessage();

    Timer time = new Timer();


    public boolean isHubActive(){
        
        if(gameData.isEmpty()){
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)){
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }
    //Shift was in active for blue if red won auto, or red if blue won auto
        boolean shift1Active = switch (alliance.get()){
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
        };

        if (matchTime > 130){
        //Transition shift hub is active
            return true;
        } else if (matchTime >105){
        //Shift 1
            return shift1Active;
        } else if (matchTime > 80){
        //Shift 2
            return !shift1Active;
        } else if (matchTime > 55){
        //Shift 3
            return shift1Active;
        } else if (matchTime > 30){
            return !shift1Active;
        } else{
        //End game, hub always active
            return true;
        }
    }
    

    private LedSubsystem(){
        //DIO Outputs
        codeChannel2 = new DigitalOutput(Constants.LEDs.CHANNEL_2_PIN);
        codeChannel3 = new DigitalOutput(Constants.LEDs.CHANNEL_3_PIN);
        codeChannel4 = new DigitalOutput(Constants.LEDs.CHANNEL_4_PIN);

        currentStatusMode = LEDStatusMode.OFF;   
    }

    public static LedSubsystem getInstance(){ 
        /**
         * method to allow calling this class and getting the single instance from anywhere, 
         * creating the instance if the first time*/ 
        if (INSTANCE == null){
            INSTANCE = new LedSubsystem();
        }
        return INSTANCE;
    }
   /**
    * OFF: Robot disabled
    * ActiveShift: 5 sec before active shift starts
    * CountdownShift: 5 seconds before active shift ends
    * HubLock: change color when hub lock active
    * CoutdownEndMatch: 15, 10, 5, 4, 3, 2, 1 tell match end
    * Default: alliance color constant / red or blue
    */ 
    public static enum LEDStatusMode{
        OFF(0),
        ActiveShift(1),
        CountdownShift(2),
        HubActive(3),
        CountdownEndGame(4),
        BlueAlliance(5),
        RedAlliance(6);
        
        private final int code;

        private LEDStatusMode(int code) {
             this.code = code;
        }
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("LED_Mode", currentStatusMode.toString());
    
        if(!disablePeriodicEval){
            //robot disabled
            if(DriverStation.isDisabled()){
                /**
                 * If not connected to station or FMS pulse default disabled color
                 */
                currentStatusMode = LEDStatusMode.OFF;
            }
            /**
            * If robot is enabled set leds to alliance color as default
            */
            if(DriverStation.isEnabled()){
                if(alliance.get() == Alliance.Red){
                    currentStatusMode = LEDStatusMode.RedAlliance;
                }
                if(alliance.get() == Alliance.Blue){
                    currentStatusMode = LEDStatusMode.BlueAlliance;
                }
                if(DriverStation.isTeleopEnabled()){
                
                    if(matchTime < 30){
                        time.start();
                        SmartDashboard.putBoolean("CountdownEndGame", matchTime < 30);
                        currentStatusMode = LEDStatusMode.CountdownEndGame;
                        if(time.get() == 0){
                            time.restart();
                        }
                    }
                    if(isHubActive() == false){
                        time.start();
                        if(time.get() > 20){
                            SmartDashboard.putBoolean("5 sec before Active", time.get()==20 && isHubActive()== false);
                            currentStatusMode = LEDStatusMode.ActiveShift;
                            if(time.get() == 2){
                                time.restart();
                            }
                         }
                    }
                    if(isHubActive() == true){
                        time.start();  
                        SmartDashboard.putBoolean("Active Hub", isHubActive() == true);
                        currentStatusMode = LEDStatusMode.HubActive;
                        if(time.get() > 20){
                            SmartDashboard.putBoolean("5 sec end to Active", time.get()==20 && isHubActive() == true);
                            currentStatusMode = LEDStatusMode.CountdownShift;
                            if(time.get() == 25){
                                time.restart();
                            }
                        }
                    } 
                }
            }
            
        }
        int code = currentStatusMode.code;

        // Code for encoding the code to binary on the digitalOutput pins
        SmartDashboard.putNumber("Sending LED Code:", code);
        codeChannel2.set((code & 1 ) > 0);
        codeChannel3.set((code & 2 ) > 0);
        codeChannel4.set((code & 4 ) > 0); 
        
    }
    
    public void setLedStatusMode(LEDStatusMode statusMode){
        if (!disablePeriodicEval){
            currentStatusMode = statusMode;
        }
    }

    public LEDStatusMode getLStatusMode(){
        return currentStatusMode;
    }

    public void clearStatusMode(){
        currentStatusMode = LEDStatusMode.OFF;
    }

    //Disables LEDs
    public void disableLEDs(){
        disablePeriodicEval = true;
    }

    //Enables LEDs
    public void enableLEDs(){
        disablePeriodicEval = false;
    }
}


// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class LedSubsystem extends SubsystemBase {

//     private static LedSubsystem INSTANCE;

//     private DigitalOutput pin2;
//     private DigitalOutput pin3;
//     private DigitalOutput pin4;

//     private LEDStatusMode currentStatusMode;

//     private boolean disablePeriodicEval = false;

//     private LedSubsystem() {
//         pin2 = new DigitalOutput(2);
//         pin3 = new DigitalOutput(3);
//         pin4 = new DigitalOutput(4);
//         currentStatusMode = LEDStatusMode.MODE_HUB_LOCK;
//     }

//     public static LedSubsystem getInstance() {
//         if (INSTANCE == null) {
//             INSTANCE = new LedSubsystem();
//         }
//         return INSTANCE;
//     }

//     /*
//      * Whatever LED status you want to have go inside this Enum
//      * Just make a new status below and add it specific code to it
//      */
//     public static enum LEDStatusMode {

//         MODE_DISABLED(0),
//         MODE_BLUE_ENABLED_NEUTRAL(1),
//         MODE_RED_ENABLED_NEUTRAL(2),
//         HUB_PREPMODE_HUB_PREPARE(3),
//         HUB_AMODE_HUB_ACTIVECTIVE(4),
//         MODE_HUB_LOCK(5),
//         MODE_END_WARNING(6);

//         /*
//          * 5 sec before shift starts/ flash tell 2 or 1 second left then go white when
//          * pre shoot
//          * 5 seconds before shift ends/ c
//          * change color when hub lock active /green
//          * 15, 10, 5, 4, 3, 2, 1 tell match end /flash allience color
//          * alliance color constant / red or blue
//          * team blue or rainbow when disabled
//          */

//         private final int code;

//         private LEDStatusMode(int code) {
//             this.code = code;
//         }
//     }

//     /*
//      * Nothing is added to the periodic yet as no LEDs have been added to the robot
//      */
//     @Override
//     public void periodic() {

//         if (!disablePeriodicEval) {
//             // currentStatusMode = LEDStatusMode.INTAKE;
//             int code = currentStatusMode.code;

//             pin2.set((code & 1) == 0);
//             pin3.set((code & 2) == 0);
//             pin4.set((code & 4) == 0);
//             SmartDashboard.putString("LEDSTATUS", currentStatusMode.toString());
//             SmartDashboard.putBoolean("LED/Pin2", (code & 1) != 0);
//             SmartDashboard.putBoolean("LED/Pin3", (code & 2) != 0);
//             SmartDashboard.putBoolean("LED/Pin4", (code & 4) != 0);
//         }

//     }

//     public void setLedStatusMode(LEDStatusMode selectedMode) {
//         if (!disablePeriodicEval) {
//             currentStatusMode = selectedMode;
//         }
//     }

//     public void clearLedStatus() {
//         if (!disablePeriodicEval) {
//             currentStatusMode = LEDStatusMode.MODE_DISABLED;
//         }
//     }

//     public void disableLEDs() {
//         disablePeriodicEval = true;
//     }

//     public void enableLEDs() {
//         disablePeriodicEval = false;
//     }

// }
