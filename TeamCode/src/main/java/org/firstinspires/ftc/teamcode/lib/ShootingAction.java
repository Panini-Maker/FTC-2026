package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.TuningVars.idle;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.shootingToleranceAuto;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniper;
import static org.firstinspires.ftc.teamcode.lib.TuningVars.sniperAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class ShootingAction {
    public DcMotorEx leftShooter, rightShooter;
    public DcMotor intake;
    public DcMotor turret;
    public Servo hoodServo, leftLatch, rightLatch;
    public ShooterController controller;

    public ShootingAction(DcMotorEx leftShooter, DcMotorEx rightShooter, DcMotor intake, DcMotor turret,
                          Servo hoodServo, Servo leftLatch, Servo rightLatch, ShooterController controller) {
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.intake = intake;
        this.turret = turret;
        this.hoodServo = hoodServo;
        this.leftLatch = leftLatch;
        this.rightLatch = rightLatch;
        this.controller = controller;
    }

    public void shoot(double shooterVelocity, int shootDurationMs, int rampUpTimeMs, double hoodAngle, double tolerance, boolean useToleranceShooting) throws InterruptedException {
        try {
            intake.setPower(0); // Ensure intake is off
        } catch (Exception e) {
            // Intake disconnected, continue
        }

        // Open latches after shooter is up to speed
        try {
            leftLatch.setPosition(0);
            rightLatch.setPosition(1);
        } catch (Exception e) {
            // Latch servos disconnected, continue
        }

        // Set hood position
        try {
            hoodServo.setPosition(hoodAngle);
        } catch (Exception e) {
            // Hood servo disconnected, continue without it
        }

        // Start shooter PID
        try {
            controller.setVelocityPID(shooterVelocity);
        } catch (Exception e) {
            // Shooter disconnected, continue
        }

        // Wait for shooter to reach target velocity (within tolerance) or timeout
        long startTime = System.currentTimeMillis();
        long maxRampUpTime = rampUpTimeMs > 0 ? rampUpTimeMs : 3500; // Default 3 seconds max
        while (System.currentTimeMillis() - startTime < maxRampUpTime) {
            try {
                double avgVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
                if (Math.abs(shooterVelocity - avgVelocity) <= tolerance) {
                    break; // Shooter is within tolerance of target
                }
            } catch (Exception e) {
                // Shooter disconnected, break out of wait loop
                break;
            }
            Thread.sleep(10);
        }

        if (!useToleranceShooting) {
            tolerance = Double.MAX_VALUE; // Ignore velocity for feeding if not using tolerance shooting
        }

        // Run intake for shoot duration, but only feed when shooter is at speed
        long shootStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - shootStartTime < shootDurationMs) {
            try {
                double avgVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
                if (Math.abs(shooterVelocity - avgVelocity) <= tolerance) {
                    try {
                        intake.setPower(1); // Safe to feed
                    } catch (Exception e) {
                        // Intake disconnected
                    }
                } else {
                    try {
                        intake.setPower(0); // Wait for shooter to recover
                    } catch (Exception e) {
                        // Intake disconnected
                    }
                }
            } catch (Exception e) {
                // Shooter disconnected, try to run intake anyway
                try {
                    intake.setPower(1);
                } catch (Exception e2) {
                    // Intake also disconnected
                }
            }
            Thread.sleep(10);
        }

        // Stop everything
        try {
            intake.setPower(0);
        } catch (Exception e) {
            // Intake disconnected
        }

        try {
            controller.setVelocityPID(idle);
        } catch (Exception e) {
            // Shooter disconnected
        }

        try {
            leftLatch.setPosition(1); // Close latches
            rightLatch.setPosition(0);
        } catch (Exception e) {
            // Latch servos disconnected
        }
    }

    public void shoot(int shooterVelocity, int shootDurationMs, int rampUpTimeMs) throws InterruptedException {
        // Set hood position
        try {
            if (shooterVelocity == (sniperAuto)) {
                hoodServo.setPosition(0.5); // Set hood for sniper
            } else {
                hoodServo.setPosition(0.42); // Set hood for shotgun
            }
        } catch (Exception e) {
            // Hood servo disconnected, continue without it
        }

        try {
            intake.setPower(0); // Ensure intake is off
        } catch (Exception e) {
            // Intake disconnected, continue
        }

        // Start shooter PID
        try {
            controller.setVelocityPID(shooterVelocity);
        } catch (Exception e) {
            // Shooter disconnected, continue
        }

        // Wait for shooter to reach target velocity (within tolerance) or timeout
        long startTime = System.currentTimeMillis();
        long maxRampUpTime = rampUpTimeMs > 0 ? rampUpTimeMs : 3500; // Default 3 seconds max
        while (System.currentTimeMillis() - startTime < maxRampUpTime) {
            try {
                double avgVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
                if (Math.abs(shooterVelocity - avgVelocity) <= shootingToleranceAuto) {
                    break; // Shooter is within tolerance of target
                }
            } catch (Exception e) {
                // Shooter disconnected, break out of wait loop
                break;
            }
            Thread.sleep(10);
        }

        // Open latches after shooter is up to speed
        try {
            leftLatch.setPosition(0);
            rightLatch.setPosition(1);
        } catch (Exception e) {
            // Latch servos disconnected, continue
        }
        Thread.sleep(2000); // Wait for latches to fully open

        // Run intake for shoot duration, but only feed when shooter is at speed
        long shootStartTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - shootStartTime < shootDurationMs) {
            try {
                double avgVelocity = (leftShooter.getVelocity() + rightShooter.getVelocity()) / 2.0;
                if (Math.abs(shooterVelocity - avgVelocity) <= 50) {
                    try {
                        intake.setPower(1); // Safe to feed
                    } catch (Exception e) {
                        // Intake disconnected
                    }
                } else {
                    try {
                        intake.setPower(0); // Wait for shooter to recover
                    } catch (Exception e) {
                        // Intake disconnected
                    }
                }
            } catch (Exception e) {
                // Shooter disconnected, try to run intake anyway
                try {
                    intake.setPower(1);
                } catch (Exception e2) {
                    // Intake also disconnected
                }
            }
            Thread.sleep(10);
        }

        // Stop everything
        try {
            intake.setPower(0);
        } catch (Exception e) {
            // Intake disconnected
        }

        try {
            controller.setVelocityPID(idle);
        } catch (Exception e) {
            // Shooter disconnected
        }

        try {
            leftLatch.setPosition(1); // Close latches
            rightLatch.setPosition(0);
        } catch (Exception e) {
            // Latch servos disconnected
        }
    }
}
