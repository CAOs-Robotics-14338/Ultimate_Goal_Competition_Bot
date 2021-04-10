package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

public class LaunchSystem {

    /* LAUNCHER */
    DcMotorEx launchLeft, launchRight;
    double launchLow  = 0.40;
    double launchHigh = 0.45;
    // TicksPerRevolution

    /* TRIGGER */
    Servo trigger;
    double trigger_extended = 0.87;
    double trigger_retracted = -0.80;



    public LaunchSystem(DcMotorEx rightLauncherMotor, DcMotorEx leftLauncherMotor, Servo triggerServo)
    {
        this.launchLeft = leftLauncherMotor;
        this.launchRight = rightLauncherMotor;
        this.trigger = triggerServo;



        launchLeft.setDirection(DcMotorEx.Direction.REVERSE);
        launchRight.setDirection(DcMotorEx.Direction.FORWARD);
        launchLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launchRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    public void launchWheelsToHIGHPower(){
        launchLeft.setVelocity(126, AngleUnit.DEGREES);
        launchRight.setVelocity(126, AngleUnit.DEGREES);
    }
    public void launchWheelsToLOWPower(){
        launchLeft.setVelocity(123.5, AngleUnit.DEGREES);
        launchRight.setVelocity(123.5, AngleUnit.DEGREES);
    }
    public void launchWheelsToAUTONOMOUSPower(double power){
        launchLeft.setVelocity(power, AngleUnit.DEGREES);
        launchRight.setVelocity(power, AngleUnit.DEGREES);
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