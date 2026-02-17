package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class AutonomousSubsystem {
    enum States{
        NOTSTARTED, SHOOTLEFT, SHOOTRIGHT,SHOOTRIGHT2ND
    };
    private States currentState = States.NOTSTARTED;

    boolean isArtifactLeftActive= false;
    boolean isArtifactRightActive= false;
    boolean isLeftShooterActive= false;
    boolean isRightShooterActive= false;
    long leftArtifactStartTime = 0;  // Track when left artifact pusher was activated
    long rightArtifactStartTime = 0;  // Track when right artifact pusher was activated
    public boolean shootGPPMotif(RobotHardware robot){
        boolean shootingFinished = false;
        if(currentState == States.NOTSTARTED)
        {
            currentState = States.SHOOTLEFT;
        }
        switch(currentState) {
            case SHOOTLEFT:
                if (!isLeftShooterActive) {
                    // Shoot left
                    robot.shooterSubsystem.pushArtifactLeft();
                    isLeftShooterActive = true;
                    isArtifactLeftActive = true;
                    leftArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactLeftActive && leftArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - leftArtifactStartTime;
                    if (elapsedTime >= 500) { // Only wait half a second before shooting the right

                        currentState = States.SHOOTRIGHT;
                    }
                }
                break;
            case SHOOTRIGHT:
                if (!isRightShooterActive) {
                    // Shoot left
                    robot.shooterSubsystem.pushArtifactRight();
                    isArtifactRightActive = true;
                    isRightShooterActive = true;
                    rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactRightActive && rightArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                    if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                        isArtifactRightActive = false;
                        isRightShooterActive = false;
                        rightArtifactStartTime = 0;
                        robot.shooterSubsystem.stopArtifactLeft();
                        isArtifactLeftActive = false;
                        isLeftShooterActive = false;
                        leftArtifactStartTime = 0;
                        currentState = States.SHOOTRIGHT2ND;
                    }
                }
                break;
            case SHOOTRIGHT2ND:
                if (!isRightShooterActive) {
                    robot.shooterSubsystem.pushArtifactRight();
                    robot.intakeSubsystem.intakeArtifactStage2();
                    isArtifactRightActive = true;
                    isRightShooterActive = true;
                    rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactRightActive && rightArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                    if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                        isArtifactRightActive = false;
                        isRightShooterActive = false;
                        rightArtifactStartTime = 0;
                        robot.shooterSubsystem.stopShooting();
                        currentState = States.NOTSTARTED;
                        robot.intakeSubsystem.stopIntake();
                        shootingFinished = true;
                    }
                }
                break;
        }
        return shootingFinished;
    }

    public boolean shootPPGMotif(RobotHardware robot){
        boolean shootingFinished = false;
        if(currentState == States.NOTSTARTED)
        {
            currentState = States.SHOOTRIGHT;
        }
        switch(currentState) {
            case SHOOTLEFT:
                if (!isLeftShooterActive) {
                    // Shoot left
                    robot.shooterSubsystem.pushArtifactLeft();
                    isLeftShooterActive = true;
                    isArtifactLeftActive = true;
                    leftArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactLeftActive && leftArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - leftArtifactStartTime;
                    if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                        isArtifactLeftActive = false;
                        isLeftShooterActive = false;
                        leftArtifactStartTime = 0;
                        robot.shooterSubsystem.stopShooting();
                        isArtifactRightActive = false;
                        isRightShooterActive = false;
                        rightArtifactStartTime = 0;
                        robot.intakeSubsystem.stopIntake();
                        currentState = States.NOTSTARTED;
                        shootingFinished = true;
                    }
                }
                break;
            case SHOOTRIGHT:
                if (!isRightShooterActive) {
                    // Shoot left
                    robot.shooterSubsystem.pushArtifactRight();
                    isArtifactRightActive = true;
                    isRightShooterActive = true;
                    rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactRightActive && rightArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                    if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                        isArtifactRightActive = false;
                        isRightShooterActive = false;
                        rightArtifactStartTime = 0;
                        currentState = States.SHOOTRIGHT2ND;
                    }
                }
                break;
            case SHOOTRIGHT2ND:
                if (!isRightShooterActive) {
                    robot.shooterSubsystem.pushArtifactRight();
                    robot.intakeSubsystem.intakeArtifactStage2();
                    isArtifactRightActive = true;
                    isRightShooterActive = true;
                    rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactRightActive && rightArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                    if (elapsedTime >= 500) {  // Wait half a second before shooting left.
                        currentState = States.SHOOTLEFT;
                    }
                }
                break;
        }
        return shootingFinished;
    }

    public boolean shootPGPMotif(RobotHardware robot){
        boolean shootingFinished = false;
        if(currentState == States.NOTSTARTED)
        {
            currentState = States.SHOOTRIGHT;
        }
        switch(currentState) {
            case SHOOTLEFT:
                if (!isLeftShooterActive) {
                    // Shoot left
                    robot.shooterSubsystem.pushArtifactLeft();
                    isLeftShooterActive = true;
                    isArtifactLeftActive = true;
                    leftArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactLeftActive && leftArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - leftArtifactStartTime;
                    if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                        robot.shooterSubsystem.stopArtifactLeft();
                        robot.intakeSubsystem.intakeArtifactStage2();
                        isArtifactLeftActive = false;
                        isLeftShooterActive = false;
                        leftArtifactStartTime = 0;
                        rightArtifactStartTime = System.currentTimeMillis();
                        currentState = States.SHOOTRIGHT2ND;
                    }
                }
                break;
            case SHOOTRIGHT:
                if (!isRightShooterActive) {
                    // Shoot left
                    robot.shooterSubsystem.pushArtifactRight();
                    isArtifactRightActive = true;
                    isRightShooterActive = true;
                    rightArtifactStartTime = System.currentTimeMillis();  // Record start time
                } else if (isArtifactRightActive && rightArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                    if (elapsedTime >= 500) {  // Wait half a second to shoot the left
                        currentState = States.SHOOTLEFT;
                    }
                }
                break;
            case SHOOTRIGHT2ND:
                if (isArtifactRightActive && rightArtifactStartTime > 0) {
                    long elapsedTime = System.currentTimeMillis() - rightArtifactStartTime;
                    if (elapsedTime >= 3000) {  // 3000 milliseconds = 3 seconds
                        isArtifactRightActive = false;
                        isRightShooterActive = false;
                        rightArtifactStartTime = 0;
                        robot.shooterSubsystem.stopShooting();
                        robot.intakeSubsystem.stopIntake();
                        currentState = States.NOTSTARTED;
                        shootingFinished = true;
                    }
                }
                break;
        }
        return shootingFinished;
    }

}
