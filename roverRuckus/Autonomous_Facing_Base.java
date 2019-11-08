/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Skystone.roverRuckus;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Facing base", group="Pushbot")
public class Autonomous_Facing_Base extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() {

        robot.init(hardwareMap,this);

        //initialize gyro
        try {

            robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
                idle();
            }
            if (robot.accesseries.color.getState()) {
                robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else {
                robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
        } catch (Exception e) {

            telemetry.log().clear();
            telemetry.log().add("Gyro FAILED!!!. Press Start.");
            telemetry.update();
        }
        //keep lift mechanism all the way down/////////////////////////////////////////////////////////////////////////////////////
        telemetry.addData("init", "Gyro Calibrated, holding up");
        telemetry.update();
        while (isStarted() == false) {

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        telemetry.addData("running", "Gyro Calibrated, running");
        telemetry.update();
        robot.accesseries.MoveBeaterToPosistion(-80, this);

        //drop from lander
        ElapsedTime timer = new ElapsedTime();//timer is in case the limit switch misses
        timer.reset();
        while (!robot.accesseries.IsAtTop() && timer.seconds() < HardwarePushbot.MAX_DOWN_TIME) {
            telemetry.addData("running", "DROPPING");
            telemetry.update();
            robot.accesseries.liftmotor.setPower(-.9);
        }
        robot.accesseries.liftmotor.setPower(0);
        //robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.BOTTOM_POSITION - 40, this);
        telemetry.addData("running", "DONE DROPPING");
        telemetry.update();

        //sense particles
        Direction GoldPosition = Direction.LEFT;
        if (!this.isStopRequested()) {
            GoldPosition = robot.FindGold(3, .5f, this);
            robot.FlowAngleDistance(0, -.3f, 2800, this);
        }
        if (GoldPosition == Direction.RIGHT) {
            GoldOnRight();
        } else if (GoldPosition == Direction.CENTER) {
            GoldOnCenter();

        } else {

            GoldOnLeft();
        }
        //end///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //  robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.BOTTOM_POSITION, this);

        robot.setMotors(0, 0);
//        while(opModeIsActive()&&!robot.accesseries.BottomSwitch.getState()) {
//            if (robot.accesseries.BottomSwitch.getState()) {
//                robot.accesseries.liftmotor.setPower(0);
//
//            } else {
//                robot.accesseries.liftmotor.setPower(.3);
//            }
//        }
        if (robot.accesseries.MoveBeaterToPosistionThread != null)
            robot.accesseries.MoveBeaterToPosistionThread.interrupt();
        robot.accesseries.liftmotor.setPower(0);

        robot.accesseries.stop();
    }
//directions switched
    private void GoldOnCenter() {
        telemetry.addData("center center center","center center center"); //displays on screen so we know what the robot sees
        telemetry.update();

        //all angles are original angles -45
        robot.FlowAngleTime(0,-.4f,2.5f,this);
       // robot.Claim();
        robot.FlowAngleDistanceAway(45,-.3f,10,this);//travels until the robot is 10cm away from the sidewall - to not scratch it
        robot.FlowAngleStrafeTime(true,.3f,1.6f,this); //strafe until in front of gold particle - the center of our robot lines up with it
       // robot.FlowAngleDistanceAway(22,-.45f,10,this);//travels until the robot is 10cm away from the sidewall - to not scratch it
//        // robot.FlowAngleTime(45,-.6f,2.3f,this,true);
//        robot.Claim(); ////beater motor spins backward to spit our claimer
//        robot.carwash(.95f);
//        robot.FlowAngleTime(-52,.4f,1.15f,this,true);//to crater
//        robot.FlowAngleStrafeTime(false,-.5f,1.3f,this);//strafe until in front of gold particle - the center of our robot lines up with it
//        robot.carwash(0);
//        robot.FlowAngleTime(-50,.5f,3.4f,this); //to crater


//        robot.Claim(); //beater motor spins backward to spit our claimer
//        robot.carwash(.95f);
//        robot.FlowAngleStrafeTime(true,.3f,1.5f,this); //strafe until in front of gold particle - the center of our robot lines up with it
//        robot.FlowAngleTime(-90,-.4f,-2.15f,this,true);//to crater
//        robot.carwash(0);
//        robot.FlowAngleTime(-76,.45f,.35f,this,true); //drives to crater
//        robot.FlowAngleTime(-58,.4f,1,this,true); //drives to crater
//        robot.FlowAngleTime(-56,.5f,3.8f,this,true); //drives to crater
        //robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.TOP_POSITION,this);

    }
    //directions switched
    private void GoldOnRight() {
        telemetry.addData("right right right","right right right"); //displays on screen so we know what the robot sees

        //all angles are original angles -45
        robot.FlowAngleTime(0,-.3f,.8f,this); //drives away from the lander to not get stuck when we strafe
        robot.FlowAngleStrafeTime(true,.3f,1.55f,this); //strafe until in front of gold particle - the center of our robot lines up with it
        robot.FlowAngleTime(0,-.3f,2.1f,this); //drives toward base
        robot.FlowAngleDistanceAway(42,-.45f,10,this);//travels until the robot is 10cm away from the sidewall - to not scratch it
       // robot.FlowAngleTime(45,-.6f,2.3f,this,true);
//        robot.Claim(); ////beater motor spins backward to spit our claimer
//        robot.carwash(.95f);
//        robot.FlowAngleStrafeTime(true,.3f,1.5f,this); //strafe until in front of gold particle - the center of our robot lines up with it
//        robot.FlowAngleTime(-52,.4f,1.2f,this,true);//to crater
//        robot.FlowAngleStrafeTime(false,.3f,1.5f,this);
//        robot.carwash(0);
//        robot.FlowAngleTime(-50,.65f,3.4f,this); //to crater
      //  robot.FlowAngleTime(-55,.85f,1.88f,this); //to crater

    }

    private void GoldOnLeft() {
        telemetry.addData("left left left","left left left");
        telemetry.update();

        //all angles are original angles -45
        robot.FlowAngleTime(0,-.3f,1.2f,this); //drives away from the lander to not get stuck when we strafe
        robot.FlowAngleStrafeTime(false,-.5f,2.1f,this);//strafe until in front of gold particle - the center of our robot lines up with it
        robot.FlowAngleTime(0,-.3f,2.6f,this); //drive toward base and move gold particle
        //hit wall
        robot.FlowAngleDistanceAway(-45,-.3f,10,this);//travels until the robot is 10cm away from the sidewall - to not scratch it
      //  robot.FlowAngleTime(45,-.6f,2.3f,this,true);//to base
//        robot.Claim();
//        robot.carwash(.95f);
//        robot.FlowAngleTime(-52,.4f,7.2f,this,true);//to ctater
//        robot.FlowAngleStrafeTime(false,-.5f,.4f,this);//strafe until in front of gold particle - the center of our robot lines up with it
//        robot.carwash(0);
//        robot.FlowAngleTime(-50,.3f,1.5f,this);
//        robot.FlowAngleTime(-60,.3f,3.2f,this); //to crater
    }
}
