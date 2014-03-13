/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.camera.AxisCamera;;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.image.NIVision.MeasurementType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 *
 * @author David Musiel
 */
public class Copperbot extends SimpleRobot {

    private Joystick leftStick = new Joystick(1);
    private Joystick rightStick = new Joystick(2);
    private Joystick xboxController = new Joystick(3);
    private Jaguar leftJag = new Jaguar(2);
    private Jaguar rightJag = new Jaguar(3);
    private Jaguar shooterMotor = new Jaguar(4);
    private Talon suction = new Talon(1);
    private Encoder rightEncoder = new Encoder(1, 2);
    private Encoder leftEncoder = new Encoder(5, 6);

    private Timer timer = new Timer();
    private SmartDashboard smartDash = new SmartDashboard();
    private SendableChooser autoChooser = new SendableChooser();

    private Compressor compressor = new Compressor(14, 1);
    //private Relay relay = new Relay(1);
    //private DigitalInput pressureSwitch = new DigitalInput(14);
    private DoubleSolenoid armSolenoid = new DoubleSolenoid(1, 2);
    private DoubleSolenoid shooterRelease = new DoubleSolenoid(3, 4);
    private DoubleSolenoid shooterPiston = new DoubleSolenoid(5, 6);
/*
    //Camera constants used for distance calculation
    final int Y_IMAGE_RES = 480;		//X Image resolution in pixels, should be 120, 240 or 480
    final double VIEW_ANGLE = 49;		//Axis M1013
    //final double VIEW_ANGLE = 41.7;		//Axis 206 camera
    //final double VIEW_ANGLE = 37.4;  //Axis M1011 camera
    final double PI = 3.141592653;

    //Score limits used for target identification
    final int RECTANGULARITY_LIMIT = 40;
    final int ASPECT_RATIO_LIMIT = 55;

    //Score limits used for hot target determination
    final int TAPE_WIDTH_LIMIT = 50;
    final int VERTICAL_SCORE_LIMIT = 50;
    final int LR_SCORE_LIMIT = 50;

    //Minimum area of particles to be considered
    final int AREA_MINIMUM = 150;

    //Maximum number of particles to process
    final int MAX_PARTICLES = 8;

    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
*/
    public boolean armUp;
    public boolean shooterUp;
    private boolean turbo;
    private boolean konamiCode;
    private double speed;

    public Copperbot() {
        super();
        armUp = true;
        shooterUp = true;
        konamiCode = true;
        speed = 0.5;
        turbo = false;
    }

    public class Scores {

        double rectangularity;
        double aspectRatioVertical;
        double aspectRatioHorizontal;
    }

    public class TargetReport {

        int verticalIndex;
        int horizontalIndex;
        boolean Hot;
        double totalScore;
        double leftScore;
        double rightScore;
        double tapeWidthScore;
        double verticalScore;
    };

    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        boolean mode = smartDash.getBoolean("Check Box 1", true);
        compressor.start();
        //cameraInit();
        //autonCamera();
        if (mode == true) {
            autoDrive();
        } else {
            autoShoot();
        }
        compressor.stop();
    }

    /**
     * This function is called each time the robot enters operator control.
     */
    public void operatorControl() {
        encoderStart();
        compressor.start();
        while (true && isOperatorControl() && isEnabled()) {
            smartDash.putBoolean("Pressure Switch", compressor.getPressureSwitchValue());
            smartDash.putBoolean("Arm Up", armUp);
            smartDash.putBoolean("Shooter Up", shooterUp);
            encoderPut();
            checkInput();
        }
        encoderStop();
        compressor.stop();
    }

    /**
     * This function is called once each time the robot enters test mode.
     */
    public void test() {

    }

    public void checkInput() {
        customDrive(leftStick.getY() * speed, rightStick.getY() * speed);
        //PID();
        if (xboxController.getRawButton(1)) { //A button
            shoot();
        }

        if (xboxController.getRawButton(3) || xboxController.getRawButton(2)) { //X and B buttons
            suction();
        } else {
            suction.set(0.0);
        }

        if (xboxController.getRawButton(5)) { //Left Bumper
            armHeight();
        }

        if (xboxController.getRawButton(6)) { //Right Bumper
            shooterHeight();
        }
        if (leftStick.getRawButton(6) || leftStick.getRawButton(7) || leftStick.getRawButton(10) || leftStick.getRawButton(11)) {
            turboSet();
        }

        if (turbo = true) {
            speed = 1;
        } else {
            speed = 0.5;
        }
    }

    public void autoDrive() {
        encoderStart();
        driveForOneEncoder(36);
        /* if (encoderDifference > 0){
         customDrive(-0.25, encoderDifference * -0.025);
         }
         else if (encoderDifference < 0){
         customDrive(-0.25, encoderDifference * 0.025);
         }
         else{
         customDrive(-0.25, -0.25);
         }*/
        encoderStop();
    }

    /**
     * Drives forward for 15 feet then stops.
     */
    public void autoShoot() {
        encoderStart();
        driveForOneEncoder(15 * 12);
        shoot();
        encoderStop();
    }

    public void autoFlop() {
        encoderStart();
        armHeight();
        suction.set(1.0);
        timer.delay(1);
        armHeight();
        timer.delay(1);
        suction.set(0.0);
        driveForOneEncoder(15 * 12);
        flop();
        encoderStop();
    }

    /**
     * Shoots the ball.
     */
    public void shoot() {
        shooterMotor.set(0.5);
        timer.delay(0.5);
        shooterRelease.set(DoubleSolenoid.Value.kReverse);
        shooterMotor.set(0.0);
        shooterRelease.set(DoubleSolenoid.Value.kForward);
        shooterRelease.set(DoubleSolenoid.Value.kOff);
    }

    public void flop() {
        suction.set(-1.0);
        timer.delay(1.5);
        suction.set(0.0);
    }

    /**
     * Controls the motor that controls the wheels that grab the ball.
     */
    public void suction() {
        if (xboxController.getRawButton(3)) {
            suction.set(1.0);
        } else if (xboxController.getRawButton(2)) {
            suction.set(-1.0);
        } else {
            suction.set(0.0);
        }
    }

    /**
     * Controls PID.
     */
    /*
     public void PID() {
     double kp = 1.00;
     double ki = 0.00;
     double kd = 4.00;
     PIDController pidControl = new PIDController(kp, ki, kp, rightEncoder, roboJag);
     pidControl.setPercentTolerance(10);
     pidControl.setContinuous();
     if (leftStick.getRawButton(3)) {
     pidControl.setSetpoint(1000);
     } else {
     pidControl.setSetpoint(0);
     }
     }
     */

    /*
     * public void autoChoose(){ autoChooser.addDefault("Driving Forward",
     * autoDrive()); autoChooser.addObject("Shooter", new autoShooter());
     * SmartDashboard.putData("Auto mode chooser,", autoChooser()); }
     */
    /**
     * Converts encoder pulses to inches.
     *
     * @param inches (inches)
     * @return the number of encoder pulses to travel the inputted distance
     */
    public double encoderConvert(double inches) {
        inches = inches * 27.25;
        return inches;
    }

    /**
     * Resets and starts both encoders.
     */
    public void encoderStart() {
        rightEncoder.reset();
        leftEncoder.reset();
        rightEncoder.start();
        leftEncoder.start();
    }

    /**
     * Resets and stops both encoders.
     */
    public void encoderStop() {
        rightEncoder.stop();
        rightEncoder.reset();
        leftEncoder.stop();
        leftEncoder.reset();
    }

    /**
     * Prints the encoder count to the smart dashboard.
     */
    public void encoderPut() {
        int rightEncoderCount = rightEncoder.get();
        int leftEncoderCount = leftEncoder.get();
        smartDash.putNumber("Right Encoder Count", rightEncoderCount);
        smartDash.putNumber("Left Encoder Count", leftEncoderCount);
    }

    /**
     * Sets the jags to go a certain speed.
     *
     * @param leftSpeed speed the left cheetah goes at.
     * @param rightSpeed speed the right lion goes at.
     */
    public void customDrive(double leftSpeed, double rightSpeed) {
        leftJag.set(leftSpeed);
        rightJag.set(rightSpeed * -1);
    }

    /**
     * Raises and lowers the robot arm.
     */
    public void armHeight() {
        if (armUp == true) {
            armSolenoid.set(DoubleSolenoid.Value.kReverse);
            timer.delay(1);
            armSolenoid.set(DoubleSolenoid.Value.kOff);
            armUp = false;
        } else if (armUp == false) {
            armSolenoid.set(DoubleSolenoid.Value.kForward);
            timer.delay(1);
            armSolenoid.set(DoubleSolenoid.Value.kOff);
            armUp = true;
        }
    }

    /**
     * Raises and lowers the shooter.
     */
    public void shooterHeight() {
        if (shooterUp = true) {
            shooterPiston.set(DoubleSolenoid.Value.kReverse);
            shooterPiston.set(DoubleSolenoid.Value.kOff);
            shooterUp = false;
        } else {
            shooterPiston.set(DoubleSolenoid.Value.kForward);
            shooterPiston.set(DoubleSolenoid.Value.kOff);
            shooterUp = true;

        }
    }

    /**
     * Drives the robot forward for 3 meters.
     */
    public void driveFor(double inches) {
        double pulse = encoderConvert(inches);
        double leftEncoderRatio = 1;
        double rightEncoderRatio = 1;
        double encoderDifference = (leftEncoder.get()) - (-1 * rightEncoder.get());
        while ((rightEncoder.get() <= pulse || leftEncoder.get() >= -1 * pulse) && isEnabled() && isAutonomous()) {
            encoderPut();
            encoderDifference = leftEncoder.get() - rightEncoder.get();
            smartDash.putNumber("Left Encoder Ratio", leftEncoderRatio);
            smartDash.putNumber("Right Encoder Ratio", rightEncoderRatio);
            smartDash.putNumber("Encoder Difference", encoderDifference);
            if (leftEncoder.get() != 0) {
                leftEncoderRatio = (rightEncoder.get() / (-1 * leftEncoder.get()));
            }

            if (rightEncoder.get() != 0) {
                rightEncoderRatio = (-1 * leftEncoder.get()) / rightEncoder.get();
            }

            if (rightEncoderRatio > 1) {
                customDrive((1 / rightEncoderRatio) * -0.25, -0.25);
            } else if (rightEncoderRatio < 1) {
                customDrive(-0.25, rightEncoderRatio * -0.25);
            } else {
                customDrive(-0.25, -0.25);
            }
        }
        customDrive(0.0, 0.0);
    }

    /**
     * As driveFor, except only uses one encoder
     *
     * @param inches
     */
    public void driveForOneEncoder(double inches) {
        double pulse = encoderConvert(inches);

        while ((rightEncoder.get() * -1 <= pulse) && isEnabled() && isAutonomous()) {
            encoderPut();
            customDrive(-0.25, -0.25);
        }
        customDrive(0.0, 0.0);

    }

    public void turboSet() {
        if (turbo == false) {
            turbo = true;
        } else {
            turbo = false;
        }
    }
/*
    public void cameraInit() {
        camera = AxisCamera.getInstance();  // get an instance of the camera
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);
    }0 

    public void autonCamera() {
        TargetReport target = new TargetReport();
        int verticalTargets[] = new int[MAX_PARTICLES];
        int horizontalTargets[] = new int[MAX_PARTICLES];
        int verticalTargetCount, horizontalTargetCount;

        while (isAutonomous() && isEnabled()) {
            try {
                /**
                 * Do the image capture with the camera and apply the algorithm
                 * described above. This sample will either get images from the
                 * camera or from an image file stored in the top level
                 * directory in the flash memory on the cRIO. The file name in
                 * this case is "testImage.jpg"
                 *
                 */
                //ColorImage image = camera.getImage();     // comment if using stored images
               /* ColorImage image;                           // next 2 lines read image from flash on cRIO
                image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
                BinaryImage thresholdImage = image.thresholdHSV(105, 137, 230, 255, 133, 183);   // keep only green objects
                //thresholdImage.write("/threshold.bmp");
                BinaryImage filteredImage = thresholdImage.particleFilter(cc);           // filter out small particles
                //filteredImage.write("/filteredImage.bmp");

                //iterate through each particle and score to see if it is a target
                Scores scores[] = new Scores[filteredImage.getNumberParticles()];
                horizontalTargetCount = verticalTargetCount = 0;

                if (filteredImage.getNumberParticles() > 0) {
                    for (int i = 0; i < MAX_PARTICLES && i < filteredImage.getNumberParticles(); i++) {
                        ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                        scores[i] = new Scores();

                        //Score each particle on rectangularity and aspect ratio
                        scores[i].rectangularity = scoreRectangularity(report);
                        scores[i].aspectRatioVertical = scoreAspectRatio(filteredImage, report, i, true);
                        scores[i].aspectRatioHorizontal = scoreAspectRatio(filteredImage, report, i, false);

                        //Check if the particle is a horizontal target, if not, check if it's a vertical target
                        if (scoreCompare(scores[i], false)) {
                            System.out.println("particle: " + i + "is a Horizontal Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                            horizontalTargets[horizontalTargetCount++] = i; //Add particle to target array and increment count
                        } else if (scoreCompare(scores[i], true)) {
                            System.out.println("particle: " + i + "is a Vertical Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                            verticalTargets[verticalTargetCount++] = i;  //Add particle to target array and increment count
                        } else {
                            System.out.println("particle: " + i + "is not a Target centerX: " + report.center_mass_x + "centerY: " + report.center_mass_y);
                        }
                        System.out.println("rect: " + scores[i].rectangularity + "ARHoriz: " + scores[i].aspectRatioHorizontal);
                        System.out.println("ARVert: " + scores[i].aspectRatioVertical);
                    }

                    //Zero out scores and set verticalIndex to first target in case there are no horizontal targets
                    target.totalScore = target.leftScore = target.rightScore = target.tapeWidthScore = target.verticalScore = 0;
                    target.verticalIndex = verticalTargets[0];
                    for (int i = 0; i < verticalTargetCount; i++) {
                        ParticleAnalysisReport verticalReport = filteredImage.getParticleAnalysisReport(verticalTargets[i]);
                        for (int j = 0; j < horizontalTargetCount; j++) {
                            ParticleAnalysisReport horizontalReport = filteredImage.getParticleAnalysisReport(horizontalTargets[j]);
                            double horizWidth, horizHeight, vertWidth, leftScore, rightScore, tapeWidthScore, verticalScore, total;

                            //Measure equivalent rectangle sides for use in score calculation
                            horizWidth = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
                            vertWidth = NIVision.MeasureParticle(filteredImage.image, verticalTargets[i], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
                            horizHeight = NIVision.MeasureParticle(filteredImage.image, horizontalTargets[j], false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);

                            //Determine if the horizontal target is in the expected location to the left of the vertical target
                            leftScore = ratioToScore(1.2 * (verticalReport.boundingRectLeft - horizontalReport.center_mass_x) / horizWidth);
                            //Determine if the horizontal target is in the expected location to the right of the  vertical target
                            rightScore = ratioToScore(1.2 * (horizontalReport.center_mass_x - verticalReport.boundingRectLeft - verticalReport.boundingRectWidth) / horizWidth);
                            //Determine if the width of the tape on the two targets appears to be the same
                            tapeWidthScore = ratioToScore(vertWidth / horizHeight);
                            //Determine if the vertical location of the horizontal target appears to be correct
                            verticalScore = ratioToScore(1 - (verticalReport.boundingRectTop - horizontalReport.center_mass_y) / (4 * horizHeight));
                            total = leftScore > rightScore ? leftScore : rightScore;
                            total += tapeWidthScore + verticalScore;

                            //If the target is the best detected so far store the information about it
                            if (total > target.totalScore) {
                                target.horizontalIndex = horizontalTargets[j];
                                target.verticalIndex = verticalTargets[i];
                                target.totalScore = total;
                                target.leftScore = leftScore;
                                target.rightScore = rightScore;
                                target.tapeWidthScore = tapeWidthScore;
                                target.verticalScore = verticalScore;
                            }
                        }
                        //Determine if the best target is a Hot target
                        target.Hot = hotOrNot(target);
                    }

                    if (verticalTargetCount > 0) {
                        //Information about the target is contained in the "target" structure
                        //To get measurement information such as sizes or locations use the
                        //horizontal or vertical index to get the particle report as shown below
                        ParticleAnalysisReport distanceReport = filteredImage.getParticleAnalysisReport(target.verticalIndex);
                        double distance = computeDistance(filteredImage, distanceReport, target.verticalIndex);
                        if (target.Hot) {
                            System.out.println("Hot target located");
                            System.out.println("Distance: " + distance);
                        } else {
                            System.out.println("No hot target present");
                            System.out.println("Distance: " + distance);
                        }
                    }
                }

                /**
                 * all images in Java must be freed after they are used since
                 * they are allocated out of C data structures. Not calling
                 * free() will cause the memory to accumulate over each pass of
                 * this loop.
                 */
               /* filteredImage.free();
                thresholdImage.free();
                image.free();

//            } catch (AxisCameraException ex) {        // this is needed if the camera.getImage() is called
//                ex.printStackTrace();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
        }
    }

    /**
     * Computes the estimated distance to a target using the height of the
     * particle in the image. For more information and graphics showing the math
     * behind this approach see the Vision Processing section of the
     * ScreenStepsLive documentation.
     *
     * @param image The image to use for measuring the particle estimated
     * rectangle
     * @param report The Particle Analysis Report for the particle
     * @param outer True if the particle should be treated as an outer target,
     * false to treat it as a center target
     * @return The estimated distance to the target in Inches.
     */
    /*double computeDistance(BinaryImage image, ParticleAnalysisReport report, int particleNumber) throws NIVisionException {
        double rectLong, height;
        int targetHeight;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        //using the smaller of the estimated rectangle long side and the bounding rectangle height results in better performance
        //on skewed rectangles
        height = Math.min(report.boundingRectHeight, rectLong);
        targetHeight = 32;

        return Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
    }

    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect
     * ratio for the target. This method uses the equivalent rectangle sides to
     * determine aspect ratio as it performs better as the target gets skewed by
     * moving to the left or right. The equivalent rectangle is the rectangle
     * with sides x and y where particle area= x*y and particle perimeter= 2x+2y
     *
     * @param image The image containing the particle to score, needed to
     * perform additional measurements
     * @param report The Particle Analysis Report for the particle, used for the
     * width, height, and particle number
     * @param outer	Indicates whether the particle aspect ratio should be
     * compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
   /* public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean vertical) throws NIVisionException {
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = vertical ? (4.0 / 32) : (23.5 / 4);	//Vertical reflector 4" wide x 32" tall, horizontal 23.5" wide x 4" tall

        //Divide width by height to measure aspect ratio
        if (report.boundingRectWidth > report.boundingRectHeight) {
            //particle is wider than it is tall, divide long by short
            aspectRatio = ratioToScore((rectLong / rectShort) / idealAspectRatio);
        } else {
            //particle is taller than it is wide, divide short by long
            aspectRatio = ratioToScore((rectShort / rectLong) / idealAspectRatio);
        }
        return aspectRatio;
    }

    /**
     * Compares scores to defined limits and returns true if the particle
     * appears to be a target
     *
     * @param scores The structure containing the scores to compare
     * @param outer True if the particle should be treated as an outer target,
     * false to treat it as a center target
     *
     * @return True if the particle meets all limits, false otherwise
     */
    /*boolean scoreCompare(Scores scores, boolean vertical) {
        boolean isTarget = true;

        isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
        if (vertical) {
            isTarget &= scores.aspectRatioVertical > ASPECT_RATIO_LIMIT;
        } else {
            isTarget &= scores.aspectRatioHorizontal > ASPECT_RATIO_LIMIT;
        }

        return isTarget;
    }

    /**
     * Computes a score (0-100) estimating how rectangular the particle is by
     * comparing the area of the particle to the area of the bounding box
     * surrounding it. A perfect rectangle would cover the entire bounding box.
     *
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    /*double scoreRectangularity(ParticleAnalysisReport report) {
        if (report.boundingRectWidth * report.boundingRectHeight != 0) {
            return 100 * report.particleArea / (report.boundingRectWidth * report.boundingRectHeight);
        } else {
            return 0;
        }
    }

    /**
     * Converts a ratio with ideal value of 1 to a score. The resulting function
     * is piecewise linear going from (0,0) to (1,100) to (2,0) and is 0 for all
     * inputs outside the range 0-2
     */
    /*double ratioToScore(double ratio) {
        return (Math.max(0, Math.min(100 * (1 - Math.abs(1 - ratio)), 100)));
    }

    /**
     * Takes in a report on a target and compares the scores to the defined
     * score limits to evaluate if the target is a hot target or not.
     *
     * Returns True if the target is hot. False if it is not.
     */
   /* boolean hotOrNot(TargetReport target) {
        boolean isHot = true;

        isHot &= target.tapeWidthScore >= TAPE_WIDTH_LIMIT;
        isHot &= target.verticalScore >= VERTICAL_SCORE_LIMIT;
        isHot &= (target.leftScore > LR_SCORE_LIMIT) | (target.rightScore > LR_SCORE_LIMIT);

       return isHot;
    }
    /*public void compressorCheck(){
     if (pressureSwitch.get()){
     relay.set(Relay.Value.kReverse);
     }
     else{
     relay.set(Relay.Value.kForward);
     }
     }*/
}
//AND THIS IS THE END
