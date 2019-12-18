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

package org.firstinspires.ftc.teamcode.Skystone.RealRobot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.roverRuckus.Direction;

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

@Autonomous(name="Auto Block Side", group="Pushbot")
public class AutonomousFacingBlocks extends LinearOpMode {
    public static final int UNDER_BRIDGE = -310;
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

        robot.init(hardwareMap,this,false,true);

        //initialize gyro
//        try {
//
//            robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//            while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
//                idle();
//            }
//            if (robot.accesseries.color.getState()) {
//                robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//            } else {
//                robot.accesseries.blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//            }
//        } catch (Exception e) {
//
//            telemetry.log().clear();
//            telemetry.log().add("Gyro FAILED!!!. Press Start.");
//            telemetry.update();
//        }
        //keep lift mechanism all the way down/////////////////////////////////////////////////////////////////////////////////////
        telemetry.addData("init", "Gyro Calibrated");
        telemetry.update();
        while (isStarted() == false) {

        }
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        telemetry.addData("running", "running");
        telemetry.update();
        robot.accessories.ArmUpDown.setTargetPosition(UNDER_BRIDGE);
        telemetry.addData("armUpDown", "dd"+robot.accessories.ArmUpDown.getCurrentPosition());
        telemetry.update();
        robot.accessories.ArmUpDown.setPower(-.4);
        robot.LeftServo.setPosition(0);
        robot.RightServo.setPosition(1);
        robot.FlowAngleDistanceAway(0,.25f,55,this);
//        Direction direction= robot.accessories.FindSkystone();
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
//        telemetry.addData("running", ""+direction.toString());
//        telemetry.update();
//        try {
//            Thread.sleep(2000);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }
        ////////////////////////////LEFT//////////////////////////////////////////////
//        if(direction==Direction.LEFT) {
//            robot.FlowAngleStrafeTime(true,.4f,.2f,this);
//        }
//        else if(direction==Direction.CENTER) {
//            robot.FlowAngleStrafeTime(false,.4f,.3f,this);
//
//        }
//        else {
//            robot.FlowAngleStrafeTime(false,.4f,1.15f,this);
//
//        }
////        stopP();
//
//
//            robot.FlowAngleDistanceAway(0, .25f, 25, this);
//            try {
//                Thread.sleep(2000);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            robot.setMotors(0, 0);


//            robot.FlowAngleTime(0, -.3f, .6f, this);
//            robot.TurnToAngle(90, .5f);
//              robot.accessories.ArmUpDown.setTargetPosition(UNDER_BRIDGE);
//              robot.accessories.ArmUpDown.setPower(-.5f);
//              robot.setSevos(1);
//            robot.FlowAngleTime(90, .4f, 1.8f, this);
//            robot.FlowAngleTime(90, -.4f, 2.2f, this);
//            //////////////////////again////////////////////////////////////////////////////////////////////////
//            //getting second Skystone///////
//            robot.TurnToAngle(0, .4f);
//            robot.FlowAngleTime(0, -.3f, .8f, this);
//            robot.FlowAngleDistanceAway(0, .25f, 25, this);
//            robot.FlowAngleTime(0, -.3f, .7f, this);
//            robot.TurnToAngle(90, .4f);
//            robot.FlowAngleTime(90, .4f, 2.4f, this);
//            robot.FlowAngleTime(90, -.4f, 1.1f, this);

        try {
                Thread.sleep(10000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            robot.setMotors(0,0,0,0);


        stop();

    }

    void stopP() {
        if (robot.accessories.tfod != null) {
            robot.accessories.tfod.activate();
        }
    }


}
