  package edu.wpi.first.wpilibj.templates;

import com.team254.lib.CheesyVisionServer;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotTemplate extends IterativeRobot {
        //basics
    CheesyVisionServer server = CheesyVisionServer.getInstance();
    public final int listenPort = 1180;
        private RobotDrive tankChassis;
        private Joystick stickLeft;
        private Joystick stickRight;
        private Joystick xboxController;
        private Compressor compressor;
        private  Relay light;
        private Relay magnet;



        //solenoid
        Solenoid extend;  //extend the pickup arm
        Solenoid retract; //retract the pickup arm
        Solenoid extend2; //extend the shooter piston
        Solenoid retract2; //retract the shooter piston

        //sensors
        Gyro gyro;
        AnalogChannel distanceSensor;
        DigitalInput Switch;




        //motor controllers
        Talon leftBox1; //used in robotdrive - left side gearbox motor 1
        Talon leftBox2; //used in robotdrive - left side gearbox motor 2
        Talon rightBox1; //used in robotdrive - right side gearbox motor 1
        Talon rightBox2; //used in robotdrive - right side gearbox motor 2
        Talon chain1; //Runs ballIntake chain

        //drive
        int Left1 = 1; //PWM port out of leftBox1 - left side gearbox motor 1
        int Left2 = 2; //PWM port out of leftBox2 - left side gearbox motor 2
        int Right1 = 3; //PWM port out of rightBox1 - right side gearbox motor 1
        int Right2 = 4; //PWM port out of rightBox2 - right side gearbox motor 2
        int ballIntake = 5; //PWM port out of chain1 - mounted on arm

        //joystick
        int stickLeftValueNumber = 2; //stickLeftValue axis number
        int stickRightValueNumber = 2; //stickRightValue axis number
        int speedLimitNumber = 3; //speedLimit axis number
        int armControlValueNumber = 3; //armControlValue axis number
        int lightValueNumber = 7; //lightValue button number
        int extendButtonNumber = 5; //extendButton button number
        int retractButtonNumber = 6; //retractButton button number
        int extendButtonNumber2 = 1;
        int retractButtonNumber2 =2;
        int magnetOnButtonNumber = 3;
        int magnetOffButtonNumber = 4;

        boolean relase = false;
        boolean shoot = false;
        int wait = 0;
        double dist;

        //controls
        int stickLeftPort = 1; //Joystick Number set in driverstation
        int stickRightPort = 2; //Joystick Number set in driverstation
        int xboxControllerPort = 3; //Controller Number set in driverstation
        int compressorPort1 = 1; //Pressure Switch Digital I/O port
        int compressorPort2 = 1; //Compressor Relay port
        int extendPort = 1; //Solenoid extend Breakout port
        int retractPort = 2; //Solenoid retract Breakout port
        int extendPort2 = 4;
        int retractPort2 = 3;
        int lightPort = 2; //LED ring Relay port
        int magnetPort = 3; //magnet port


        //vars
        double horizontalCord = 0;
        double verticalCord = 0;
        int count = 0;
        boolean startcount = false;
    public void robotInit() {
        server.setPort(listenPort);
        server.start();
        stickLeft = new Joystick(stickLeftPort);
        stickRight = new Joystick(stickRightPort);
        xboxController = new Joystick(xboxControllerPort);
        compressor = new Compressor(compressorPort1,compressorPort2);
        extend = new Solenoid(extendPort);
        retract = new Solenoid(retractPort);
        extend2 = new Solenoid(extendPort2);
        retract2 = new Solenoid(retractPort2);
        light = new Relay(lightPort);

        compressor.start();
        distanceSensor = new AnalogChannel(1);
        Switch = new DigitalInput(6);


        magnet = new Relay(magnetPort);
        light.set(Relay.Value.kReverse);

        magnet.set(Relay.Value.kForward);
        extend.set(false);
        retract.set(true);
        extend2.set(true);
        retract2.set(false);




        leftBox1 = new Talon(Left1);
        leftBox2 = new Talon(Left2);
        rightBox1 = new Talon(Right1);
        rightBox2 = new Talon(Right2);
        chain1 = new Talon(ballIntake);



        boolean startcount = false;


        tankChassis = new RobotDrive(leftBox1, leftBox2, rightBox1, rightBox2);







    }
    public void shootauton(){

            if(startcount==true){
                tankChassis.drive(0, 0);

             retract2.set(true);
             extend2.set(false);
            count++;
            }
             if(count ==20){
                 chain1.set(1);
                 magnet.set(Relay.Value.kReverse);
                 startcount = false;



                }
    }


    public void load(){
        retract2.set(false);
        extend2.set(true);
        startcount = false;
        magnet.set(Relay.Value.kForward);
        count = 0;
    }


    public void autonomousPeriodic() {
       wait++;
        SmartDashboard.putNumber("wait", wait);

        double dist = distanceSensor.getVoltage();

        SmartDashboard.putNumber("dist", dist);

        retract.set(true);
        extend.set(false);

         if(dist >= .51){
             wait = 0;
             count=0;
             magnet.set(Relay.Value.kForward);

                tankChassis.tankDrive(-.8, .8);



            }
         if(wait <= 30){
             extend2.set(true);
             retract2.set(false);
         }
         if(wait >= 30){
         if(dist <= .51){
             if(server.getLeftStatus()){

             startcount = true;
             shoot = true;
             shootauton();
         }}}


            SmartDashboard.putInt("count", count);










//
//
//


    }








    public void teleopPeriodic() {
        SmartDashboard.putDouble("dist",distanceSensor.getVoltage());
        SmartDashboard.putBoolean("switch", Switch.get());
            tankChassis.setSafetyEnabled(true);

            double stickLeftValue = stickLeft.getRawAxis(stickLeftValueNumber);
            double stickRightValue = stickRight.getRawAxis(stickRightValueNumber);
            double speedLimit = -stickRight.getRawAxis(speedLimitNumber);
            double armControlValue = -xboxController.getRawAxis(armControlValueNumber);
            boolean lightValue = xboxController.getRawButton(lightValueNumber);
            boolean extendButton = xboxController.getRawButton(extendButtonNumber);
            boolean retractButton = xboxController.getRawButton(retractButtonNumber);
            boolean extendButton2 = xboxController.getRawButton(extendButtonNumber2);
            boolean retractButton2 = xboxController.getRawButton(retractButtonNumber2);
            boolean magnetOn = xboxController.getRawButton(magnetOnButtonNumber);
            boolean magnetOff= xboxController.getRawButton(magnetOffButtonNumber);
            double cameraVertical = -xboxController.getRawAxis(1);
            double cameraHorizontal = -xboxController.getRawAxis(2);


            speedLimit = speedLimit + 1;
            speedLimit = speedLimit / 2;
            cameraHorizontal = cameraHorizontal +1;
            cameraHorizontal = cameraHorizontal /2;
            cameraVertical = cameraVertical +1;
            cameraVertical = cameraVertical /2;


            if(extendButton2 == true){
                extend.set(true);
                retract.set(false);
                System.out.println("extendButton pressed");
            }
            if(retractButton2 ==true){
                retract.set(true);
                extend.set(false);
                System.out.println("retractButton pressed");
            }
            tankChassis.setMaxOutput(speedLimit);

            if(extendButton == true){
                if(relase == true){
                    magnet.set(Relay.Value.kReverse);
                 relase = false;
                }
            }
            if(xboxController.getRawAxis(6)  <= -.2){
                relase = true;

            }
            if(retractButton){
                load();
            }
            if(relase == true){
                retract2.set(true);
                extend2.set(false);

            }


            chain1.set(armControlValue);
            tankChassis.tankDrive(stickLeftValue, stickRightValue, true);






    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {




    }

}
