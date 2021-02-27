package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoal {
    // Set up Hardware Devices
        /* Motors */
    DcMotor linSlide;
    double slidePower = 1.0;
        /*Servos*/
    Servo claw;
    /*** CLAW Positions ***/
    double neutral = 0;
    double grab = 0.5;
    double store = 1;

    //Constructors
    public WobbleGoal(DcMotor linearSlide, Servo servo){
        linSlide = linearSlide;
         claw = servo;
    }
    //Methods
    /***SERVO METHODS***/

    public void setClaw(double pos) {
        this.claw.setPosition(pos);
    }

    public void storeClaw(){
        this.setClaw(store);
    }
    public void activateClaw() {
        this.setClaw(grab);
    }
    public void neturalClaw(){
        this.setClaw(neutral);
    }
    public void zeroPosition(){
        this.setClaw(0);
    }
    /*** LINEAR SLIDE METHODS ***/
    public void raiseWobbleGoal(){
        linSlide.setPower(slidePower);
    }
    public void lowerWobbleGoal(){
        linSlide.setPower(-1*slidePower);
    }
    public void stopGoal(){ linSlide.setPower(0);}
    public void deliverWobbleGoal(){
        //lower linear slide  timed??
        //move claw to its stored position
    }
}