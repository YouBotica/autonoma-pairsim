/* 
Copyright 2024 Purdue AI Racing

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at:

    http://www.apache.org/licenses/LICENSE-2.0

The software is provided "AS IS", WITHOUT WARRANTY OF ANY KIND, 
express or implied. In no event shall the authors or copyright 
holders be liable for any claim, damages or other liability, 
whether in action of contract, tort or otherwise, arising from, 
out of or in connection with the software or the use of the software.
*/
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using VehicleDynamics;

namespace Autonoma
{
public class ImuSimulator : MonoBehaviour
{

    public Vector3 imuGyro; // [rad/s]
    public Vector3 imuAccel; // [m/s^2]
    public Vector3 imuAngle; // [deg]
    public Vector3 imuVelLocal,imuVelLocalPrev;
    public Rigidbody rb;
    public NoiseGenerator headingNoiseGenerator;
    public NoiseGenerator gyroNoiseGenerator;
    public NoiseGenerator accelNoiseGenerator;
    public NoiseGenerator velNoiseGenerator;

    void Start()
    {
        rb = HelperFunctions.GetParentComponent<Rigidbody>(transform);

        //Gaussian Noise Simulators
        float headingMean = GameManager.Instance.Settings.mySensorSet.headingMean;
        float headingVariance = GameManager.Instance.Settings.mySensorSet.headingVariance;
        int headingSeed = GameManager.Instance.Settings.mySensorSet.headingSeed;
        headingNoiseGenerator = new NoiseGenerator(headingMean, headingVariance, headingSeed);

        float gyroMean = GameManager.Instance.Settings.mySensorSet.gyroMean;
        float gyroVariance = GameManager.Instance.Settings.mySensorSet.gyroVariance;
        int gyroSeed = GameManager.Instance.Settings.mySensorSet.gyroSeed;
        gyroNoiseGenerator = new NoiseGenerator(gyroMean, gyroVariance, gyroSeed);

        float velMean = GameManager.Instance.Settings.mySensorSet.velMean;
        float velVariance = GameManager.Instance.Settings.mySensorSet.velVariance;
        int velSeed = GameManager.Instance.Settings.mySensorSet.velSeed;
        velNoiseGenerator = new NoiseGenerator(velMean, velVariance, velSeed);

        float accelMean = GameManager.Instance.Settings.mySensorSet.accelMean;
        float accelVariance = GameManager.Instance.Settings.mySensorSet.accelVariance;
        int accelSeed = GameManager.Instance.Settings.mySensorSet.accelSeed;
        accelNoiseGenerator = new NoiseGenerator(accelMean, accelVariance, accelSeed);

    }
    void FixedUpdate()
    {   
        
        Vector3 localAngularvelocity = transform.InverseTransformDirection(rb.angularVelocity);
        imuGyro = HelperFunctions.unity2vehDynCoord(-localAngularvelocity); 
        imuVelLocal = HelperFunctions.unity2vehDynCoord( transform.InverseTransformDirection( rb.GetPointVelocity( transform.position ) ) ); 
        Vector3 dvdt = (imuVelLocal - imuVelLocalPrev)/Time.fixedDeltaTime;
        Vector3 localGravity = transform.InverseTransformDirection(Physics.gravity);
        imuAccel = dvdt - Vector3.Cross(imuVelLocal,imuGyro) - HelperFunctions.unity2vehDynCoord(localGravity); 

        // Add Gaussian noise to IMU Gyro
        float gyroNoiseX = (float)gyroNoiseGenerator.NextGaussian();
        float gyroNoiseY = (float)gyroNoiseGenerator.NextGaussian();
        float gyroNoiseZ = (float)gyroNoiseGenerator.NextGaussian();
        imuGyro[0] += gyroNoiseX;
        imuGyro[1] += gyroNoiseY;
        imuGyro[2] += gyroNoiseZ;

        // Add Gaussian noise to IMU Accel
        float accelNoiseX = (float)accelNoiseGenerator.NextGaussian();
        float accelNoiseY = (float)accelNoiseGenerator.NextGaussian();
        float accelNoiseZ = (float)accelNoiseGenerator.NextGaussian();
        imuAccel[0] += accelNoiseX;
        imuAccel[1] += accelNoiseY;
        imuAccel[2] += accelNoiseZ;

        // Add Gaussian noise to IMU Vel
        float velNoiseX = (float)velNoiseGenerator.NextGaussian();
        float velNoiseY = (float)velNoiseGenerator.NextGaussian();
        float velNoiseZ = (float)velNoiseGenerator.NextGaussian();
        imuVelLocal[0] += velNoiseX;
        imuVelLocal[1] += velNoiseY;
        imuVelLocal[2] += velNoiseZ;

        imuVelLocalPrev = imuVelLocal;

        // euler angles; some sensors output it with their internal fusion algorithms.
        imuAngle = transform.eulerAngles;
        for(int i = 0; i<3; i++)
        {
            imuAngle[i] = imuAngle[i] > 180f ? imuAngle[i] - 360f : imuAngle[i];
        }

        // RPY, RHS +, [deg], NORTH = 0 for yaw, EAST = -90, [-180,180]
        imuAngle = HelperFunctions.unity2vehDynCoord(-imuAngle); 

        // Add Gaussian noise to IMU Angle
        float headingNoiseX = (float)headingNoiseGenerator.NextGaussian();
        float headingNoiseY = (float)headingNoiseGenerator.NextGaussian();
        float headingNoiseZ = (float)headingNoiseGenerator.NextGaussian();
        imuAngle[0] += headingNoiseX;
        imuAngle[1] += headingNoiseY;
        imuAngle[2] += headingNoiseZ;
        

    }


}
}