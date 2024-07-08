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
using System;

public class GnssSimulator : MonoBehaviour
{
    public Vector3 antennaVelGlobal;
    public Vector3 antennaPosGlobal;
    public Rigidbody rb;
    public double[] llh = {0.0,0.0,0.0};
    public double lat,lon,height;
    public double lat0,lon0,h0;    
    public float vE, vN, vU;
    public TrackParams trackParams;
    public NoiseGenerator posNoiseGenerator;
    public NoiseGenerator velNoiseGenerator;
    void Start() 
    {
        rb = HelperFunctions.GetParentComponent<Rigidbody>(transform);
        try
        {
            lat0 = GameManager.Instance.Settings.myTrackParams.LAT_ORIGIN;
            lon0 = GameManager.Instance.Settings.myTrackParams.LON_ORIGIN;
            h0 = GameManager.Instance.Settings.myTrackParams.HEIGHT_ORIGIN;
        }
        catch 
        {
            Debug.Log("lat0 lon0 not defined!");
            
            if(trackParams != null)
            {
                lat0 = trackParams.LAT_ORIGIN;
                lon0 = trackParams.LAT_ORIGIN;
                h0 = trackParams.HEIGHT_ORIGIN;
            }
        }

        float posMean = GameManager.Instance.Settings.mySensorSet.posMean;
        float posVariance = GameManager.Instance.Settings.mySensorSet.posVariance;
        int posSeed = GameManager.Instance.Settings.mySensorSet.posSeed;
        posNoiseGenerator = new NoiseGenerator(posMean, posVariance, posSeed);

        float velMean = GameManager.Instance.Settings.mySensorSet.velMean;
        float velVariance = GameManager.Instance.Settings.mySensorSet.velVariance;
        int velSeed = GameManager.Instance.Settings.mySensorSet.velSeed;
        velNoiseGenerator = new NoiseGenerator(velMean, velVariance, velSeed);

    } 
    void FixedUpdate()
    {   
        // E N U , RPY
        antennaPosGlobal = HelperFunctions.unity2enu(transform.position);
        llh = Enu2LatLonHeight.calcLatLonHeight(antennaPosGlobal,lat0,lon0,h0);
        lat = llh[0];
        lon = llh[1];
        height = llh[2];
        
        antennaVelGlobal = HelperFunctions.unity2enu( rb.GetPointVelocity( transform.position ) );
        vE = antennaVelGlobal[0];
        vN = antennaVelGlobal[1];
        vU = antennaVelGlobal[2];

        // Add Gaussian noise to GNSS Vel
        float velNoiseE = (float)velNoiseGenerator.NextGaussian();
        float velNoiseN = (float)velNoiseGenerator.NextGaussian();
        float velNoiseU = (float)velNoiseGenerator.NextGaussian();
        vE += velNoiseE;
        vN += velNoiseN;
        vU += velNoiseU;

        // Add Gaussian noise to GNSS lat Lon [m -> lat/lon]
        float latNoise= (float)posNoiseGenerator.NextGaussian() * 0.00001f;
        float lonNoise = (float)posNoiseGenerator.NextGaussian() * 0.00001f;
        float heightNoise= (float)posNoiseGenerator.NextGaussian();
        lat += latNoise;
        lon += lonNoise;
        height += heightNoise;


    }

    public static (ushort, uint) GetGPSWeekAndMS()
    {
        double epoch = 3657.0 * 24.0 * 60.0 * 60.0; //offset to gps 
        double week_s = 604800;

        // mimic unix time secs from 1970
        TimeSpan t = (DateTime.UtcNow - new DateTime(1970, 1, 1)); //linux 
        double gps_time = t.TotalSeconds + 18.0 - epoch;
        ushort weeks = (ushort)(gps_time / week_s);
        double rem_seconds = gps_time - (weeks * week_s);
        uint week_ms = (uint)(rem_seconds * 1000.0);

        return (weeks, week_ms);
    }
}

