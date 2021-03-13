package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.concurrent.TimeUnit;

public class LaunchSystem {

    /* LAUNCHER */
    DcMotor launchLeft, launchRight;
    double launchLow  = 0.40;
    double launchHigh = 0.45;

    /* TRIGGER */
    Servo trigger;
    double trigger_extended = 0.76;
    double trigger_retracted = -0.86;



    public LaunchSystem(DcMotor rightLancherMotor, DcMotor leftLauncherMotor, Servo triggerServo){
        this.launchLeft = leftLauncherMotor;
        this.launchRight = rightLancherMotor;
        this.trigger = triggerServo;

        launchLeft.setDirection(DcMotor.Direction.REVERSE);
        launchRight.setDirection(DcMotor.Direction.FORWARD);
    }

    public void launchWheelsToHIGHPower(){
        launchLeft.setPower(launchHigh);
        launchRight.setPower(launchHigh);
    }
    public void launchWheelsToLOWPower(){
        launchRight.setPower(launchLow);
        launchLeft.setPower(launchLow);
    }
    public void noLaunchWheels(){
        launchLeft.setPower(0);
        launchRight.setPower(0);
    }

    public void triggerLaunch(){
        trigger.setPosition(trigger_extended);
    }
    public void triggerBack(){
        trigger.setPosition(trigger_retracted);
    }

}