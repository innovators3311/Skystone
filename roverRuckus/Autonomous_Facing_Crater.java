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

package org.firstinspires.ftc.teamcode.roverRuckus;

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

@Autonomous(name="Auto Facing Crater", group="Pushbot")
public class Autonomous_Facing_Crater extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePushbot         robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap,this);

        //initialize gyro
        try {

            while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
                idle();
            }
        }
        catch(Exception e){

            telemetry.log().clear();
            telemetry.log().add("Gyro FAILED!!!. Press Start.");
            telemetry.update();
        }

        //keep lift mechanism all the way down/////////////////////////////////////////////////////////////////////////////////////
        telemetry.addData("init","Gyro Calibrated, holding up");
        telemetry.update();

        while(isStarted()==false){

        }////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        telemetry.addData("running","Gyro Calibrated, running");
        telemetry.update();
       //
        // robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.MIDDLE_POSITION,this);

        //drop from lander
        ElapsedTime timer = new ElapsedTime();//timer is in case the limit switch misses
        timer.reset();
        while( !robot.accesseries.IsAtTop() && timer.seconds()<HardwarePushbot.MAX_DOWN_TIME) {
            robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
            robot.accesseries.liftmotor.setPower(-.9);
            telemetry.addData("Robot is Lowered", "Top Switch"+robot.accesseries.IsAtTop());
            telemetry.update();
        }
        robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        telemetry.addData("Top Boi", "Lowered"+robot.accesseries.IsAtTop());
        telemetry.update();
        //robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.BOTTOM_POSITION-40,this);
        robot.accesseries.liftmotor.setPower(0);
        //sense particles
        Direction GoldPosition=Direction.LEFT;
        if (!this.isStopRequested()) {
            GoldPosition = robot.FindGold(2, .5f, this);
        }


       robot.FlowAngleDistance(0,-.3f,-900,this);
       //right//////////////////////////////////////////
       if(GoldPosition==Direction.RIGHT)
       {
           GoldOnRight();
        }
        //center/////////////////////
        else if(GoldPosition==Direction.CENTER)
        {
            GoldOnCenter();
        }
        else {
           GoldOnLeft();
       }
        //end///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        //robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.BOTTOM_POSITION,this);

        robot.setMotors(0,0);
//        while(opModeIsActive()&&!robot.accesseries.BottomSwitch.getState()) {
//            if (robot.accesseries.BottomSwitch.getState()) {
//                robot.accesseries.liftmotor.setPower(0);
//
//            } else {
//                robot.accesseries.liftmotor.setPower(.3);
//            }
//        }
        robot.accesseries.liftmotor.setPower(0);
        if(robot.accesseries.MoveBeaterToPosistionThread!=null)
            robot.accesseries.MoveBeaterToPosistionThread.interrupt();

        robot.accesseries.stop();

    }
//speeds changed for new motors, used to be 0.8
    private void GoldOnLeft() {

        //TESTTESTTEST
        telemetry.addData("running", "LEFT LEFT LEFT");
        telemetry.update();
        robot.FlowAngleStrafeTime(false,.37f,1.5f,this);
        robot.FlowAngleTime(0,-.3f,.89f,this);
        robot.FlowAngleTime(0,.3f,.79f,this);
        robot.FlowAngleTime(60,-.25f,2.8f,this);
        robot.FlowAngleTime(140,-.3f,2,this);
        robot.FlowAngleDistanceAway(135,-.3f,40,this);//base
        robot.Claim();
        robot.carwash(.95f);
        robot.FlowAngleStrafeTime(true,.3f,.6f,this);
        robot.carwash(0f);
        robot.FlowAngleTime(130,.5f,4.8f,this);
    }

    //speeds changed for new motors, used to be 0.8
    private void GoldOnCenter() {
        //based on LEFT
        telemetry.addData("running", "CENTER CENTER CENTER");
        telemetry.update();
        robot.FlowAngleTime(0,-.3f,1.2f,this); //hit gold
        robot.FlowAngleTime(0,.4f,.8f,this);
        robot.FlowAngleStrafeTime(false,.4f,1.8f,this);
       // robot.FlowAngleStrafeTime(false,.4f,.6f,this);
        robot.FlowAngleTime(80,-.3f,3,this,true);
        robot.FlowAngleTime(140,-.3f,2.1f,this,true);
        robot.FlowAngleDistanceAway(135,-.3f,20,this);//base
        robot.Claim();
        robot.Claim();
        robot.carwash(.95f);
        robot.FlowAngleStrafeTime(true,.3f,.3f,this);
        robot.carwash(0);
        robot.FlowAngleTime(135,.55f,4.2f,this);

    }

    //speeds changed for new motors, used to be 0.8
    private void GoldOnRight() {
        //based on LEFT
        telemetry.addData("running", "RIGHT RIGHT RIGHT");
        telemetry.update();
        robot.FlowAngleStrafeTime(true,.3f,2.4f,this);
        robot.FlowAngleTime(0,-.3f,.98f,this);
        robot.FlowAngleTime(0,.3f,.78f,this);
        robot.FlowAngleStrafeTime(false,.48f,4.2f,this);
        robot.FlowAngleTime(70,-.3f,3,this,true);
        robot.FlowAngleTime(140,-.4f,2.1f,this,true);
        robot.FlowAngleDistanceAway(135,-.5f,40,this);//base
        robot.Claim();
        robot.Claim();
        robot.Claim();
        robot.carwash(.95f);
        robot.FlowAngleStrafeTime(true,.3f,.3f,this);
        robot.carwash(0);
        robot.FlowAngleTime(135,.5f,4.5f,this);
        }

//MEAN CRATER
//    private void GoldOnCenter() {
//        telemetry.addData("running","CENTER CENTER CENTER");
//        telemetry.update();
//        robot.FlowAngleDistance(0,-.6f,-3000,this);
//        robot.FlowAngleDistance(0,.6f,1500,this);
//        robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//        robot.FlowAngleTime(90,.6f,2.2f,this);
//        robot.FlowAngleTime(55,.6f,2.5f,this,true);
//        robot.FlowAngleTime(0,.6f,2.5f,this);
//        robot.FlowAngleTime(-55,.8f,4.2f,this);//to base
////           robot.FlowAngleTime(-45,.6f,2.5f,this);//to base
////           robot.FlowAngleTime(-90,.8f,5,this);//to other team's crater while passing over their particles
//        robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.TOP_POSITION,this);
//    }


//MEAN CRATER
//    private void GoldOnRight() {
//        telemetry.addData("running","RIGHT RIGHT RIGHT");
//        telemetry.update();
//        robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        robot.FlowAngleStrafeTime(true,.5f,1.5f,this);
//        robot.FlowAngleTime(0,-.6f,1,this);
//        robot.FlowAngleTime(-20,-.6f,1.5f,this);
//        robot.FlowAngleTime(-135,.6f,2.5f,this);
//        robot.FlowAngleTime(135,-.6f,3.5f,this,true);
//        robot.accesseries.MoveBeaterToPosistion(HardwarePushbot.TOP_POSITION,this);



//           robot.Claim();
//           robot.FlowAngleTime(90,.6f,5,this);
//            robot.Claim();
//           robot.FlowAngleDistance(0,-.6f,-4,this);
//           robot.FlowAngleDistance(-20,-.6f,-3,this);
//    }
}
