classdef SpotSensor < Simulink.IntEnumType
    enumeration
        xRedPhasespace       (SpotCoord.xRed)
        yRedPhasespace       (SpotCoord.yRed)
        thetaRedPhasespace   (SpotCoord.thetaRed)
        xBlackPhasespace     (SpotCoord.xBlack)
        yBlackPhasespace     (SpotCoord.yBlack)
        thetaBlackPhasespace (SpotCoord.thetaBlack)
        xBluePhasespace      (SpotCoord.xBlue)
        yBluePhasespace      (SpotCoord.yBlue)
        thetaBluePhasespace  (SpotCoord.thetaBlue)
        shoulderArmEncoder   (SpotCoord.shoulderArm)
        elbowArmEncoder      (SpotCoord.elbowArm)
        wristArmEncoder      (SpotCoord.wristArm)
%
%       % offset of 12 corresponds to the number of PhaseSpace measurements
        xRedRatePhasespace       (12 + SpotCoord.xRed)
        yRedRatePhasespace       (12 + SpotCoord.yRed)
        thetaRedRatePhasespace   (12 + SpotCoord.thetaRed)
        xBlackRatePhasespace     (12 + SpotCoord.xBlack)
        yBlackRatePhasespace     (12 + SpotCoord.yBlack)
        thetaBlackRatePhasespace (12 + SpotCoord.thetaBlack)
        xBlueRatePhasespace      (12 + SpotCoord.xBlue)
        yBlueRatePhasespace      (12 + SpotCoord.yBlue)
        thetaBlueRatePhasespace  (12 + SpotCoord.thetaBlue)
        shoulderArmRateEncoder   (12 + SpotCoord.shoulderArm)
        elbowArmRateEncoder      (12 + SpotCoord.elbowArm)
        wristArmRateEncoder      (12 + SpotCoord.wristArm)
%
%       % offset of 12 corresponds to the number of measured rates
        xRedImu       (12 + 12 + SpotCoord.xRed)
        yRedImu       (12 + 12 + SpotCoord.yRed)
        thetaRedImu   (12 + 12 + SpotCoord.thetaRed)
        xBlackImu     (12 + 12 + SpotCoord.xBlack)
        yBlackImu     (12 + 12 + SpotCoord.yBlack)
        thetaBlackImu (12 + 12 + SpotCoord.thetaBlack)
        xBlueImu      (12 + 12 + SpotCoord.xBlue)
        yBlueImu      (12 + 12 + SpotCoord.yBlue)
        thetaBlueImu  (12 + 12 + SpotCoord.thetaBlue)
%
%       % offset of 9 corresponds to the number of IMU measurements
        xStereo     (12 + 12 + 9 + 1)
        yStereo     (12 + 12 + 9 + 2)
        thetaStereo (12 + 12 + 9 + 3)
        xLidar      (12 + 12 + 9 + 4)
        yLidar      (12 + 12 + 9 + 5)
        thetaLidar  (12 + 12 + 9 + 6)
        rLaser      (12 + 12 + 9 + 7)
    end
end

