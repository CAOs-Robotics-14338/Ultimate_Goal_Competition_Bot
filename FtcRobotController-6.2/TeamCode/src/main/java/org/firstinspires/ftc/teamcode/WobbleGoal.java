package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

public class WobbleGoal {
    // Set up Hardware Devices
    /* Motor */
    DcMotor linSlide;
    double slideLowerPower = 0.25; //1.0
    double slideLiftPower = 0.80;
    /*Servos*/
    Servo claw;
    /*** CLAW Positions ***/
    double lift = 0.5;
    double grab = 1;
    double stow = 0;

    //Constructors
    // Associate the motor and servo with the class
    public WobbleGoal(DcMotor linearSlide, Servo servo){
        linSlide = linearSlide;
        claw = servo;
        linSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.storeClaw();
    }
    //Methods

    /***SERVO METHODS***/
    public void setClaw(double pos) {
        this.claw.setPosition(pos);
    }
    public void storeClaw(){
        this.setClaw(stow);
    } //Claw is in its upright position: Beginning of Autonomous
    public void activateClaw() {
        this.setClaw(grab);
    }//Claw is in the position to grab the wobble goal from the ground
    public void setliftClawPosition(){
        this.setClaw(lift);
    }//the lift position tilts the wobble goal off of the ground for quick movement
    public void zeroPosition(){
        this.setClaw(0);
    } //Used for testing purposes

    /*** LINEAR SLIDE METHODS ***/
    public void raiseWobbleGoal(){
        linSlide.setPower(slideLiftPower);
    }
    public void lowerWobbleGoal(){
        linSlide.setPower(-1*slideLowerPower);
    }
    public void stopGoal(){ linSlide.setPower(0);}
    public void deliverWobbleGoal(){
        //autonomous??
        //lower linear slide timed, encodered, ....??
        //move claw to its stored position
    }
}