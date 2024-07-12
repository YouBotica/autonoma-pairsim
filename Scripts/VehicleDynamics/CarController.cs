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

using UnityEngine;
using VehicleDynamics;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;

public class CarController : MonoBehaviour
{
    public VehicleParameters vehicleParams;
    public Rigidbody carBody;
    public float throttleCmd, brakeCmd, steerAngleCmd; // controller inputs
    public float steerAngleApplied, steerAngleAppliedPrev; // after the actuator dyn
    public float brakeApplied, brakeAppliedPrev ,thrApplied, thrAplliedPrev;  // after the actuator dyn
    private float[] steerAngleCmdBuf;
    private float[] steerAngleCmdBufPrev;
    private float[] brakeCmdBuf;
    private float[] brakeCmdBufPrev;
    public bool gearUp, gearDown , gearUpPrev, gearDownPrev;
    public float omegaRR,omega1,omega2;
    public int gear = 1;
    public float TEngine,TEnginePrev,rpmEngine;
    public float TAxle,TBrake;
    public Vector3 frontDownforce,frontDownforceGlobal;
    public Vector3 rearDownforce,rearDownforceGlobal;
    public Vector3 dragForce,dragForceGlobal;
    public Vector3 V;
    public bool physicalActuator = false;
    public VehicleState vehicleState;
    public Powertrain powertrain;
    public NoiseGenerator steerNoiseGenerator;
    public NoiseGenerator brakeNoiseGenerator;
    public NoiseGenerator throttleNoiseGenerator;
    // private string dataFilePath; //for changing gaussian noise on-the-fly
    void getState()
    {
        // take values from unity system and transform into VD coords
        V = HelperFunctions.unity2vehDynCoord( transform.InverseTransformDirection(carBody.velocity) );

        Vector3 pose = HelperFunctions.unity2vehDynCoord(-transform.eulerAngles);
        pose.x = Mathf.Abs(pose.x) > 180f ? pose.x - Mathf.Sign(pose.x)*360f : pose.x;
        pose.y = Mathf.Abs(pose.y) > 180f ? pose.y - Mathf.Sign(pose.y)*360f : pose.y;
        pose.z = Mathf.Abs(pose.z) > 180f ? pose.z - Mathf.Sign(pose.z)*360f : pose.z;
        
        Vector3 localAngularvelocity = transform.InverseTransformDirection(carBody.angularVelocity);
        Vector3 carAngularVel = HelperFunctions.unity2vehDynCoord(-localAngularvelocity);
        Vector3 position = HelperFunctions.unity2vehDynCoord(transform.position);
        vehicleState.pos[0] = position.x;
        vehicleState.pos[1] = position.y;
        vehicleState.pos[2] = position.z;
        vehicleState.V[0] = V.x;
        vehicleState.V[1] = V.y;
        vehicleState.V[2] = V.z;
        vehicleState.rollRate = carAngularVel.x;
        vehicleState.pitchRate = carAngularVel.y;
        vehicleState.yawRate = carAngularVel.z;
        vehicleState.roll = pose.x;
        vehicleState.pitch = pose.y;
        vehicleState.yaw = pose.z;
        vehicleState.omega[0] = omega1;
        vehicleState.omega[1] = omega2;
        vehicleState.omega[2] = omegaRR;
        vehicleState.omega[3] = omegaRR;    
    }

    void calcWheelStAngle()
    {   
        if (physicalActuator) 
        {
            // Debug.Log("Physical Actuator Used for Calculating Steering Angle");
            steerAngleCmdBufPrev = steerAngleCmdBuf;
            steerAngleCmdBuf = HelperFunctions.pureDelay(steerAngleCmd,steerAngleCmdBufPrev, vehicleParams.steeringDelay);
            steerAngleApplied = steerAngleCmdBuf[vehicleParams.steeringDelay-1];
            // Apply low pass filter and rate limiting to prevent abrupt changes in the applied steering angle
            steerAngleApplied = HelperFunctions.lowPassFirstOrder(steerAngleApplied,steerAngleAppliedPrev,vehicleParams.steeringBandwidth);
            steerAngleApplied = HelperFunctions.rateLimit(steerAngleApplied, steerAngleAppliedPrev , Mathf.Abs(vehicleParams.steeringRate/vehicleParams.steeringRatio)); //steerAngleApplied [rad]
            
            // Add Gaussian noise
            float steerNoise = (float)steerNoiseGenerator.NextGaussian();
            steerAngleApplied += steerNoise;

            // Debug.Log("Steer Noise: " + steerNoise);

            steerAngleAppliedPrev = steerAngleApplied;
        }
        else
        {
            Debug.Log("Physical Actuator NOT used for Calculating Steering Angle");
            steerAngleApplied = steerAngleCmd;
        }
    }

    void calcBrakeTorque()
    {   
        if (physicalActuator)
        {
            brakeCmdBufPrev = brakeCmdBuf;
            brakeCmdBuf = HelperFunctions.pureDelay(brakeCmd,brakeCmdBufPrev, vehicleParams.brakeDelay); // buffer the incoming commands
            brakeApplied = brakeCmdBuf[vehicleParams.brakeDelay-1]; // select the latest of the buffer for the delay
            brakeApplied = HelperFunctions.lowPassFirstOrder(brakeApplied,brakeAppliedPrev, vehicleParams.brakeBandwidth);
            brakeApplied = HelperFunctions.rateLimit(brakeApplied, brakeAppliedPrev , vehicleParams.brakeRate); //brakeApplied [%]

            // Add Gaussian noise
            float brakeNoise = (float)brakeNoiseGenerator.NextGaussian();
            brakeApplied += brakeNoise;

            brakeAppliedPrev = brakeApplied;
            TBrake = brakeApplied * vehicleParams.brakeKpaToNm;
        }
        else
        {   brakeApplied = brakeCmd;
            TBrake = brakeCmd * vehicleParams.brakeKpaToNm;
        }
    }

    void gearShifts()
    {
        if (gearUp && !gearUpPrev && gear < vehicleParams.numGears)
            gear++;
        if (gearDown && !gearDownPrev && gear > 0)
            gear--;  

        gear = Mathf.Clamp(gear,1,vehicleParams.numGears);    

        gearUpPrev = gearUp;
        gearDownPrev = gearDown;
    }

    void calcEngineTorque()
    {        
        //TEngine = throttleCmd * HelperFunctions.lut1D(VehParams.engineMapLen,VehParams.engineMapRevs,VehParams.engineMapTorque,rpmEngine);
        float saturatedRpmEngine = Mathf.Clamp(rpmEngine,vehicleParams.minEngineMapRpm,vehicleParams.maxEngineRpm);
        thrApplied = HelperFunctions.lowPassFirstOrder(throttleCmd,thrAplliedPrev, vehicleParams.throttleBandwidth);
        thrApplied = HelperFunctions.rateLimit(thrApplied, thrAplliedPrev , vehicleParams.throttleRate);

        // Add Gaussian noise
        float throttleNoise = (float)throttleNoiseGenerator.NextGaussian();
        thrApplied += throttleNoise;
            
        thrAplliedPrev = thrApplied;

        float torquePercentage = HelperFunctions.lut1D(vehicleParams.numPointsThrottleMap,
                                vehicleParams.throttleMapInput, vehicleParams.throttleMapOutput, thrApplied);

        TEngine = torquePercentage *( (float)(vehicleParams.enginePoly[0]*Mathf.Pow(saturatedRpmEngine,2) 
                + vehicleParams.enginePoly[1]*saturatedRpmEngine + vehicleParams.enginePoly[2]) + vehicleParams.engineFrictionTorque);
        if (rpmEngine > 1050)
        {
            TEngine = TEngine - vehicleParams.engineFrictionTorque;
        }
        if (TEngine > TEnginePrev )
        {
            TEngine = HelperFunctions.rateLimit(TEngine, TEnginePrev , vehicleParams.torqueRate);
        }
        TAxle = TEngine * vehicleParams.gearRatio[gear-1];
        TEnginePrev= TEngine;
    }
    void applyAeroForces()
    {
        float aeroForce = -0.6f*vehicleParams.Af*V.x*V.x;
        frontDownforce.y  = aeroForce*vehicleParams.ClF;
        rearDownforce.y = aeroForce*vehicleParams.ClR;
        dragForce.z = aeroForce*vehicleParams.Cd*Mathf.Sign(V.x);
        frontDownforceGlobal = transform.TransformDirection(frontDownforce);
        rearDownforceGlobal  = transform.TransformDirection(rearDownforce);
        dragForceGlobal = transform.TransformDirection(dragForce);

        carBody.AddForceAtPosition(frontDownforceGlobal, transform.TransformPoint(vehicleParams.frontDownforcePos));
        carBody.AddForceAtPosition(rearDownforceGlobal, transform.TransformPoint(vehicleParams.rearDownforcePos));
        carBody.AddForceAtPosition(dragForceGlobal, transform.TransformPoint(vehicleParams.dragForcePos));
    } 
    
    void Start()
    {
        UpdateVehicleParamsFromMenu();

        initializeBuffers();
        carBody = GetComponent<Rigidbody>();
        carBody.mass = vehicleParams.mass;
        carBody.inertiaTensor = vehicleParams.Inertia;
        carBody.centerOfMass = vehicleParams.centerOfMass;

        //Gaussian Noise Simulators
        float steerMean = GameManager.Instance.Settings.mySensorSet.steerMean;
        float steerVariance = GameManager.Instance.Settings.mySensorSet.steerVariance;
        int steerSeed = GameManager.Instance.Settings.mySensorSet.steerSeed;
        steerNoiseGenerator = new NoiseGenerator(steerMean, steerVariance, steerSeed);

        float brakeMean = GameManager.Instance.Settings.mySensorSet.brakeMean;
        float brakeVariance = GameManager.Instance.Settings.mySensorSet.brakeVariance;
        int brakeSeed = GameManager.Instance.Settings.mySensorSet.brakeSeed;
        brakeNoiseGenerator = new NoiseGenerator(brakeMean, brakeVariance, brakeSeed);

        float throttleMean = GameManager.Instance.Settings.mySensorSet.throttleMean;
        float throttleVariance = GameManager.Instance.Settings.mySensorSet.throttleVariance;
        int throttleSeed = GameManager.Instance.Settings.mySensorSet.throttleSeed;
        throttleNoiseGenerator = new NoiseGenerator(throttleMean, throttleVariance, throttleSeed);

        string aeroConfigFileName = "AeroParams.json";
        string fullAeroPath = Path.Combine(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Personal), "PAIRSIM_config"), "Parameters/" + aeroConfigFileName);
        CheckAndCreateDefaultFile(fullAeroPath, defaultAeroParams);

        string suspensionConfigFileName = "SuspensionParams.json";
        string fullSuspensionPath = Path.Combine(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.Personal), "PAIRSIM_config"), "Parameters/" + suspensionConfigFileName);
        CheckAndCreateDefaultFile(fullSuspensionPath, defaultSuspensionParams);

        LoadAeroParametersFromJson(fullAeroPath);
        LoadSuspensionParametersFromJson(fullSuspensionPath);

    }

    [System.Serializable]
    public class AeroParametersConfig
    {
        // ------------------Aerodynamics --------------------------
        public double Af;// = 1; // [m^2] Frontal Area
        public double ClF;// = 0.65f; // downforce coef. at the front axle
        public double ClR;// = 0.5f*1.6f/1.3f; // downforce coef. at the rear axle
        public Vector3 frontDownforcePos;
        public Vector3 rearDownforcePos;
        public double Cd;// = 0.81f; 
        public Vector3 dragForcePos;// = new Vector3(0f,0.2f,0f); // [m] center of pressure for the drag force, above the origin point of the chassis
    }

    [System.Serializable]
    public class SuspensionParametersConfig
    {
        // --------------- Suspension -----------------------
        public double kArbFront;
        public double kArbRear;
        public double kSpring;// = 200000.0f; // [N/m] carrying 800kg car with 10cm deflection
        public double cDamper;// = 4000.0f; //8855 [Ns/m] for zeta = 0.707 = c/(2*sqrt(km))
        public double lSpring;// = 0.3f; // [m]
    }

    private AeroParametersConfig defaultAeroParams = new AeroParametersConfig
    {
        Af = 1.0, // [m^2] 
        ClF = 0.65, // downforce coef. at the front axle
        ClR = 1.18,// // downforce coef. at the rear axle
        frontDownforcePos = new Vector3(0.0f, 0.0f, 1.7f),
        rearDownforcePos = new Vector3(0.0f, 0.0f, -1.3f),
        Cd = 0.8581, 
        dragForcePos = new Vector3(0.0f, 0.0f, 0.0f) // [m] center of pressure for the drag force, above the origin point of the chassis
    };
    private SuspensionParametersConfig defaultSuspensionParams = new SuspensionParametersConfig
    {
        kArbFront = 463593.0,
        kArbRear = 358225.0,
        kSpring = 200000.0, // [N/m] 
        cDamper = 8000.0,// [Ns/m] 
        lSpring = 0.3//[m]
    };

    void LoadAeroParametersFromJson(string filePath)
    {
        string fullPath = Path.Combine(Application.dataPath, filePath);
        if (File.Exists(fullPath))
        {
            string jsonData = File.ReadAllText(fullPath);
            AeroParametersConfig Params = JsonUtility.FromJson<AeroParametersConfig>(jsonData);
            ApplyAeroParameters(Params);
        }
        else
        {
            Debug.LogError("Aerodynamic parameters file not found: " + fullPath);
        }
    }
    void LoadSuspensionParametersFromJson(string filePath)
    {
        string fullPath = Path.Combine(Application.dataPath, filePath);
        if (File.Exists(fullPath))
        {
            string jsonData = File.ReadAllText(fullPath);
            SuspensionParametersConfig Params = JsonUtility.FromJson<SuspensionParametersConfig>(jsonData);
            ApplySuspensionParameters(Params);
        }
        else
        {
            Debug.LogError("Aerodynamic parameters file not found: " + fullPath);
        }
    }

    void ApplyAeroParameters(AeroParametersConfig config)
    {
        vehicleParams.Af = (float)config.Af;
        vehicleParams.ClF = (float)config.ClF;
        vehicleParams.ClR = (float)config.ClR;
        vehicleParams.frontDownforcePos = config.frontDownforcePos;
        vehicleParams.rearDownforcePos = config.rearDownforcePos;
        vehicleParams.Cd = (float)config.Cd;
        vehicleParams.dragForcePos = config.dragForcePos;
    }
    void ApplySuspensionParameters(SuspensionParametersConfig config)
    {
        vehicleParams.kArbFront = (float)config.kArbFront;
        vehicleParams.kArbRear = (float)config.kArbRear;
        vehicleParams.kSpring = (float)config.kSpring; 
        vehicleParams.cDamper = (float)config.cDamper;
        vehicleParams.lSpring = (float)config.lSpring;

    }
    //check to see if params file already exists, if not write the file
    void CheckAndCreateDefaultFile<T>(string filePath, T defaultParams)
    {
        if (!File.Exists(filePath))
        {
            Directory.CreateDirectory(Path.GetDirectoryName(filePath));
            string json = JsonUtility.ToJson(defaultParams, true);
            File.WriteAllText(filePath, json);
        }
    }
    void UpdateVehicleParamsFromMenu()
    {
        vehicleParams.brakeKpaToNm = GameManager.Instance.Settings.myVehSetup.BrakeConstant;
        vehicleParams.kArbFront = GameManager.Instance.Settings.myVehSetup.FrontRollBarRate;
        vehicleParams.kArbRear = GameManager.Instance.Settings.myVehSetup.RearRollBarRate;
        vehicleParams.maxSteeringAngle = (GameManager.Instance.Settings.myVehSetup.MaxSteeringAngle > 0) ? GameManager.Instance.Settings.myVehSetup.MaxSteeringAngle : 200f;
        vehicleParams.rearSolidAxle = !GameManager.Instance.Settings.myVehSetup.IsLSD;
        vehicleParams.steeringBandwidth = (GameManager.Instance.Settings.myVehSetup.SteeringBW > 0) ? GameManager.Instance.Settings.myVehSetup.SteeringBW : 5f;
        vehicleParams.steeringDelaySec = (GameManager.Instance.Settings.myVehSetup.SteeringDelay > 0) ? GameManager.Instance.Settings.myVehSetup.SteeringDelay : 0.01f;
        vehicleParams.steeringRate = (GameManager.Instance.Settings.myVehSetup.MaxSteeringRate > 0) ? GameManager.Instance.Settings.myVehSetup.MaxSteeringRate : 360f;
        vehicleParams.steeringRatio = -1f*Mathf.Abs(GameManager.Instance.Settings.myVehSetup.SteeringRatio);
        vehicleParams.tAmb = GameManager.Instance.Settings.myVehSetup.AmbientTemp;
        vehicleParams.tTrack = GameManager.Instance.Settings.myVehSetup.TrackTemp;
        physicalActuator = !GameManager.Instance.Settings.myVehSetup.IsIdealSteering;
        vehicleParams.calcDepVars();
    }

    void initializeBuffers()
    {
        steerAngleCmdBuf = new float[vehicleParams.steeringDelay];
        steerAngleCmdBufPrev = new float[vehicleParams.steeringDelay];
        brakeCmdBuf = new float[vehicleParams.brakeDelay];
        brakeCmdBufPrev = new float[vehicleParams.brakeDelay];
    }
    
    void Update() 
    {         
        Debug.DrawRay(transform.TransformPoint(vehicleParams.frontDownforcePos), frontDownforceGlobal*0.001f, Color.yellow);
        Debug.DrawRay(transform.TransformPoint(vehicleParams.rearDownforcePos), rearDownforceGlobal*0.001f, Color.red);
        Debug.DrawRay(transform.TransformPoint(vehicleParams.dragForcePos), dragForceGlobal*0.001f, Color.green);
    }
    
    void FixedUpdate()
    {
        getState();
        gearShifts();
        calcWheelStAngle();
        calcBrakeTorque();
        calcEngineTorque();
        applyAeroForces();
    }

    public float GetSpeed()
    {
        float[] v = vehicleState.V;
        return new Vector3(v[0], v[1], v[2]).magnitude;
    }
}
