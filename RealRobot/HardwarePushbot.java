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
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWE
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Skystone.RealRobot;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import java.io.File;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
enum Direction
{
    LEFT, CENTER, RIGHT;
}
public class HardwarePushbot
{
    private static final String VUFORIA_KEY = "AXTF/LP/////AAABmS/x33JhJkAynrU/ggLtC+5u5UI4i6vN/nZ7SlkLSyJY0DibuV6b/MqkjFB3Bg8y1BxP7ODkNV31aw64HP19MxyVGC4mMf7bTAnb7ORwgRebgahyUezUi+z3LCQO0FrciWVbFG8MrlupKvtZllPW3pH5pO0OJljn7kA1jjuKpQhikG6+TUOIAejFIPHLxGUyBk6i+w3MhxN6hFbX5NMjHbFIV4fbTc0yC8CUK6mSXdlrMt4dE89taWBOSaP5UOXNTbws5N5+84U4eJFjFi1AFsNJOUDYC4XQOoIjH9lb9pCy8p7egNJXJoOPeooW7cfpqRcfLnOetqNpdvgTadNE2ARLaL6pGBR6iXq6FM0sYzvV";
    public static final float MAX_DOWN_TIME = 3.2f;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    public static final int MIDDLE_POSITION =-420;  //-420
    public static final int TOP_POSITION =-1080; //-1080
    public static final int BOTTOM_POSITION =-50; //-50
    public static final int ARM_IN=0;
    public static final int ARM_OUT=-1680;
    public LinearOpMode OpMode;
    private DistanceSensor FrontRange, LeftRange, RightRange;
    Rev2mDistanceSensor sensorTimeOfFlightFront;
    Rev2mDistanceSensor sensorTimeOfFlightRightSide;
    Rev2mDistanceSensor sensorTimeOfFlightLeftSide;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    Servo  LeftServo;
    Servo  RightServo;

    /* Public OpMode members. */
   BNO055IMU imu;
    public HardwareAccessories accessories;
   // public Servo lights = null;
    public DcMotor  leftFrontDrive   = null, leftBackDrive=null;
    public DcMotor  rightFrontDrive  = null,rightBackDrive=null;
    Orientation             lastAngles = new Orientation();

    double globalAngle;
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbot(){

    }



    /**
     * Initialize the Vuforia localization engine.
     */
        private void initVuforia() {
            /*
             * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
             */
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = hwMap.get(WebcamName.class, "Webcam 1");

            //  Instantiate the Vuforia engine
            vuforia = ClassFactory.getInstance().createVuforia(parameters);
        }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod(float confidence) {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        //tfodParameters.useObjectTracker = false;
        // set the minimumConfidence to a higher percentage to be more selective when identifying objects.
  //      tfodParameters.minimumConfidence = confidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

//
//    public boolean setMotors(double Left, double Right)
//    {
//        if(leftFrontDrive==null ||rightFrontDrive==null )
//            return false;
//        leftFrontDrive.setPower(Left*.7);
//        leftBackDrive.setPower(Left*.7);
//
//        rightFrontDrive.setPower(Right*.7);
//        rightBackDrive.setPower(Right*.7);
//        return true;
//    }

    public void setMotors (double lf, double lb, double rf, double rb){
//        if (userubberbanding= true)
//        {
//            rf_target = rf;
//            rb_target = rb;
//            lf_target = lf;
//            lb_target = lb;
//        }
//        else
        {
            leftFrontDrive.setPower(lf);
            leftBackDrive.setPower(lb);

            rightFrontDrive.setPower(rf);
            rightBackDrive.setPower(rb);
//
//            rightFrontMotor.setPower(rf);
//            leftFrontMotor.setPower(lf);
//            rightBackMotor.setPower(rb);
//            leftBackMotor.setPower(lb);
        }
    }



    public void init(HardwareMap ahwMap, LinearOpMode parent, boolean initializedTfod) {
        init(ahwMap,parent, true, initializedTfod);
    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, LinearOpMode parent, boolean ResetGyro,boolean initializeTfod) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        OpMode = parent;

        try {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

            parameters.mode = BNO055IMU.SensorMode.IMU;
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled = false;

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hwMap.get(BNO055IMU.class, "imu");

            int AXIS_MAP_CONFIG_BYTE=0x18;
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal&0x0F);
            Thread.sleep(100);
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE&0x0F);
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_CONFIG_BYTE&0x0F);
            imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal&0x0F);
            Thread.sleep(100);

            imu.initialize(parameters);
            if(ResetGyro){
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();


                String filename = "AdafruitIMUCalibration.json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
            }
            else {
                parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode

            }
//            rangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "ranger");
            leftFrontDrive = hwMap.get(DcMotor.class, "lf");
            leftBackDrive = hwMap.get(DcMotor.class, "lb");
            rightFrontDrive = hwMap.get(DcMotor.class, "rf");
            rightBackDrive = hwMap.get(DcMotor.class, "rb");

            FrontRange = hwMap.get(DistanceSensor.class, "front_range");
            sensorTimeOfFlightFront = (Rev2mDistanceSensor)FrontRange;
            RightRange = hwMap.get(DistanceSensor.class, "rside_range");
            sensorTimeOfFlightRightSide = (Rev2mDistanceSensor)RightRange;
            LeftRange = hwMap.get(DistanceSensor.class, "lside_range");
            sensorTimeOfFlightLeftSide = (Rev2mDistanceSensor)LeftRange;

            LeftServo = hwMap.get(Servo.class, "left_s");
            RightServo = hwMap.get(Servo.class, "right_s");


            blinkinLedDriver = hwMap.get(RevBlinkinLedDriver.class, "lights");
            pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
            blinkinLedDriver.setPattern(pattern);


            accessories = new HardwareAccessories();
            accessories.init(hwMap,parent,initializeTfod);
            // Set all motors to zero power
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD); ////Wires are flipped///// //reversing/////////////
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE); /////Wires are flipped///////reversing///////
        }

        catch(Exception e){

        }

    }
    public void FlowAngleTime(float DesiredAngle, float speed, float time_s, LinearOpMode opmode)
    {
        FlowAngleTime(DesiredAngle,speed,time_s,opmode,false);
    }
    public void FlowAngleTime(float DesiredAngle, float speed, float time_s, LinearOpMode opmode,boolean tight)
    {
        ElapsedTime     runtime = new ElapsedTime();
        runtime.reset();
        while (opmode.opModeIsActive() && (runtime.seconds() < time_s)) {

            float currentAngle= (float) getAngle();
            float AngleError= (float) (currentAngle-DesiredAngle);

            if(tight){
               AngleError=AngleError/60f;
           }
           else{
               AngleError=AngleError/100f;
           }
            if(speed+(.5*AngleError)>1){
                AngleError=AngleError/2f;

            }

            float left  = speed+AngleError;
            float right = speed-AngleError;

            // Normalize the values so neither exceed +/- 1.0
            float max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }
            setMotors(left,left,right,right);
            opmode.idle();

            OpMode.telemetry.addData("FlowAngleTime", "currentAngle "+currentAngle+" "+left +" "+right);
            OpMode.telemetry.update();
        }
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotors(0,0,0,0);
    }

    public void FlowAngleDistanceAway(float DesiredAngle, float speed, float distance, LinearOpMode opmode)
    {
        opmode.telemetry.log().add("FlowAngle Dist", "starting");
        opmode.telemetry.update();
        int offset=0;// leftFrontDrive.getCurrentPosition();
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //does not work for backwards yet!!!
        if(speed<0 || distance<0)
        {
            Log.wtf("Drive" , "Not supporting negative values");
        }


            //  int loops=0;
            while (opmode.opModeIsActive() &&(sensorTimeOfFlightFront.getDistance(DistanceUnit.CM)>distance)) {
                //  opmode.telemetry.log().add("Counts %d Target %d real %.1f", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getCurrentPosition() - offset, distance_counts);
                // opmode.telemetry.update();
                float AngleError = (float) (getAngle() - DesiredAngle);

                //loops++;

                AngleError = AngleError /80f;
                if (speed + (.5 * AngleError) > 1) {
                    AngleError = AngleError / 8f;
                }

                //is this the right direction
                float left = speed + AngleError;
                float right = speed - AngleError;

                // Normalize the values so neither exceed +/- 1.0
                float max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0) {
                    left /= max;
                    right /= max;
                }
                setMotors(left,left,right,right);
                opmode.idle();
                opmode.telemetry.addData("FlowAngle Dist", "starting");
                opmode.telemetry.addData("FlowAngle Dist2", "starting11"+ sensorTimeOfFlightFront.getDistance(DistanceUnit.CM));
                opmode.telemetry.update();

            }
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        setMotors(0,0,0,0);
    }

    public void FlowAngleDistance(float DesiredAngle, float speed, float distance_rot, LinearOpMode opmode)
    {
        //3000 counts = 9 inches/////////////////////////////////////
        //333 counts per inch
        opmode.telemetry.log().add("FlowAngle Dist11", "starting");
        opmode.telemetry.update();
        int offset=0;// leftFrontDrive.getCurrentPosition();
        float distance_counts=distance_rot;
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //does not work for backwards yet!!!
        if(speed<0 || distance_rot<0)
        {
            Log.wtf("Drive" , "Not supporting negative values");
        }

        if(speed<0|| distance_rot<0)
        {
            opmode.telemetry.addData("working on it %d",this.leftFrontDrive.getCurrentPosition() - offset);
            opmode.telemetry.update();
          //  int loops=0;
            while (opmode.opModeIsActive() &&(this.leftFrontDrive.getCurrentPosition() - offset> distance_counts)) {
              //  opmode.telemetry.log().add("Counts %d Target %d real %.1f", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getCurrentPosition() - offset, distance_counts);
               // opmode.telemetry.update();
                float AngleError = (float) (getAngle() - DesiredAngle);

                //loops++;

                AngleError = AngleError / 20f;
                if (speed + (.5 * AngleError) > 1) {
                    AngleError = AngleError / 2f;
                }

                //is this the right direction
                float left = speed + AngleError;
                float right = speed - AngleError;

                // Normalize the values so neither exceed +/- 1.0
                float max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0) {
                    left /= max;
                    right /= max;
                }
                setMotors(left,left,right, right);
                opmode.idle();
            }

        }
        else {
            opmode.telemetry.addData("ELSE", "Counts %d Target %d real %.1f ", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getCurrentPosition() - offset, distance_counts);
            opmode.telemetry.update();
            while (opmode.opModeIsActive() && ((this.leftFrontDrive.getCurrentPosition() - offset) < distance_counts)) {
                opmode.telemetry.addData("FlowAngle Dist", "Countsoo %d Target %d real %.1f ", leftFrontDrive.getCurrentPosition(), leftFrontDrive.getCurrentPosition() - offset, distance_counts);
                opmode.telemetry.update();
                float AngleError = (float) (getAngle() - DesiredAngle);

                AngleError = AngleError / 20f;
                if (speed + (.5 * AngleError) > 1) {
                    AngleError = AngleError / 2f;
                }

                float left = speed + AngleError;
                float right = speed - AngleError;

                // Normalize the values so neither exceed +/- 1.0
                float max = Math.max(Math.abs(left), Math.abs(right));
                if (max > 1.0) {
                    left /= max;
                    right /= max;
                }
                setMotors(left,left,right,right);
                opmode.idle();
            }
        }
        setMotors(0,0,0,0);
    }

    public void FlowAngleStrafeTime( boolean StrafeLeft, float speed, float time_s, LinearOpMode opmode)
    {
        double compass_ideal_heading=-1;
        ElapsedTime     runtime = new ElapsedTime();
        runtime.reset();
        compass_ideal_heading=getAngle();
        while (opmode.opModeIsActive() && (runtime.seconds() < time_s)) {

            if(StrafeLeft) {
                double angle_difference = (compass_ideal_heading - getAngle()) / 200.0;
                leftBackDrive.setPower(.5 - angle_difference);
                leftFrontDrive.setPower(-.5 - angle_difference);
                rightBackDrive.setPower(-.5 + angle_difference);
                rightFrontDrive.setPower(.5 + angle_difference);
            }

            else{
                double angle_difference=(compass_ideal_heading-getAngle())/200.0;
                leftBackDrive.setPower(-.5-angle_difference);
                leftFrontDrive.setPower(.5-angle_difference);
                rightBackDrive.setPower(.5+angle_difference);
                rightFrontDrive.setPower(-.5+angle_difference);
            }
            opmode.idle();
        }
        setMotors(0,0,0,0);
    }
    public double getAngle()
    {
        try{
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;}

        catch(Exception e){
            OpMode.telemetry.log().add("FlowAngle Dist11", e.toString());
            OpMode.telemetry.update();
            try {
                Thread.sleep(5000);
            } catch (InterruptedException ex) {
                ex.printStackTrace();
            }
            return 0;
        }
    }
    final float TURNSPEED=.01f;
    public void TurnToAngle(float targetangle, float speed) {
        double Difference=0;
        Difference=getAngle()-targetangle;

        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OpMode.telemetry.addData("turn to angle", "starting"+Difference+" "+getAngle()+" "+targetangle);
        OpMode.telemetry.update();
        while(Math.abs(Difference)>2&&!OpMode.isStopRequested()){
            Difference=getAngle()-targetangle;
            float MotorSpeed= (float) Math.abs(Difference*TURNSPEED);
            OpMode.telemetry.addData("turn to angle", "working"+Difference+" "+MotorSpeed+" ");
            OpMode.telemetry.update();
            if(MotorSpeed<.2){
                MotorSpeed=.2f;
            }
            if(Difference<0){
                //left turn
                setMotors(-1*MotorSpeed,-1*MotorSpeed,MotorSpeed,MotorSpeed);
            }
            else{
                //right turn
                setMotors(MotorSpeed,MotorSpeed, -1*MotorSpeed,-1*MotorSpeed);
            }
        }while(Math.abs(Difference)>2&&!OpMode.isStopRequested()){
            Difference=getAngle()-targetangle;
            float MotorSpeed= (float) Math.abs(Difference*TURNSPEED);
            OpMode.telemetry.addData("turn to angle", "working"+Difference+" "+MotorSpeed+" ");
            OpMode.telemetry.update();
            if(MotorSpeed<.2){
                MotorSpeed=.2f;
            }
            if(Difference<0){
                //left turn
                setMotors(-1*MotorSpeed,-1*MotorSpeed,MotorSpeed,MotorSpeed);
            }
            else{
                //right turn
                setMotors(MotorSpeed,MotorSpeed ,-1*MotorSpeed,-1*MotorSpeed);
            }
        }
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setMotors(0,0,0,0);

        OpMode.telemetry.addData("turn to angle", "ending"+Difference+" "+getAngle()+" "+targetangle);
        OpMode.telemetry.update();

    }

    public void setSevos(float servoPosition) {
        this.LeftServo.setPosition(servoPosition);
        this.RightServo.setPosition(servoPosition);
    }
}

