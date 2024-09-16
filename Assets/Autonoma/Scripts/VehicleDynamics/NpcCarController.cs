using UnityEngine;
using VehicleDynamics;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System;


public class NpcCarController : MonoBehaviour
{
    public Rigidbody carBody;
    // public Rigidbody egoCarBody;
    
    public bool recievedGhostPosition = false;
    private bool hasUpdatedGhostParameters = false;

    private Vector3 targetPosition;  // Current target position for the car
    private float targetHeading;
    private bool isMoving = false; 
    private float currentSpeed = 0f;  // Current speed of the car

    void Start()
    {
        carBody = GetComponent<Rigidbody>();
        // egoCarBody = GetComponent<Rigidbody>();

    }

    void Update()
    {
        if (isMoving)
        {
            targetPosition = HelperFunctions.vehDynCoord2Unity(targetPosition); //convert ghost_frame CRS (ENU??) -> Unity CRS

            float targetHeadingDegrees = Mathf.Rad2Deg * targetHeading;
            float unityHeading = -targetHeadingDegrees + 90.0f;  // Transform heading from ENU to Unity (LHS)
            Quaternion targetRotation = Quaternion.Euler(0, unityHeading, 0);

            if(recievedGhostPosition & !hasUpdatedGhostParameters){
                
                //make rigidbody behave kinematically
                carBody.isKinematic = true; 

                string targetLayerName = "Default";
                int currentLayer = gameObject.layer;
                int targetLayer = LayerMask.NameToLayer(targetLayerName);
                if (currentLayer >= 0 && targetLayer >= 0)
                {
                    // Disable collision between the current object's layer and the target layer
                    Physics.IgnoreLayerCollision(currentLayer, targetLayer, true);
                    Debug.Log($"Disabled collision between layer {LayerMask.LayerToName(currentLayer)} and layer {targetLayerName}");
                }
                else
                {
                    Debug.LogError("Invalid layer names or indices.");
                }

                // Disable Wheel scripts
                disableWheels();

                hasUpdatedGhostParameters = true;
            }

            DirectSetStates(targetPosition, targetRotation);


        }
    }
    
    // Set the target position for the car to move towards, called from GhostVehicleSubscriber.cs
    public void SetTargetPosition(Vector3 position)
    {
        targetPosition = position; //in veh dynamics frame (RH)
        isMoving = true;  // Start moving
    }

    public void SetTargetHeading(float heading)
    {
        targetHeading = heading; //in veh dynamics frame (RH)
    }

    // Directly set the car's position
    public void DirectSetStates(Vector3 newPosition, Quaternion newHeading)
    {
        // if (recievedGhostPosition & !hasInitializedPosition){
        //     Debug.Log("Shifting vertical position of Npc car and adjusting kinematic values");
        //     newPosition.y = egoCarBody.position.y;
        //     // Debug.Log("is kinematics?: " + isKinematicBool);
        //     carBody.isKinematic = isKinematicBool;
        //     hasInitializedPosition = true;
        // }

        // newPosition.y = egoCarBody.position.y;
        carBody.position = newPosition;  // Directly set Rigidbody's position
        carBody.velocity = Vector3.zero;  // Reset velocity
        transform.position = newPosition;  // Update transform position as well

        carBody.transform.rotation = newHeading;

        ConformToTerrain(); //only works with enabled collision

        isMoving = false;  // Stop any movement
    }

    // Function to adjust the car's rotation to match the terrain's slope and conform to the terrain vertically
    private void ConformToTerrain()
    {
        RaycastHit hit;
        // Perform a raycast downwards to detect the terrain and adjust rotation
        if (Physics.Raycast(transform.position + Vector3.up, Vector3.down, out hit))
        {
            // Align the car's rotation with the terrain's normal
            Quaternion targetRotation = Quaternion.FromToRotation(transform.up, hit.normal) * transform.rotation;
            carBody.MoveRotation(Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * 5f));

            // Adjust the car's position to be on the terrain surface
            Vector3 adjustedPosition = transform.position;
            adjustedPosition.y = hit.point.y;  // Set the vertical position to match the terrain height
            carBody.position = adjustedPosition;
        }
    }

    public void disableWheels()
    {
        Transform frontLeftWheel = transform.Find("Wheels/FrontLeft");
        Transform frontRightWheel = transform.Find("Wheels/FrontRight");
        Transform rearLeftWheel = transform.Find("Wheels/RearLeft");
        Transform rearRightWheel = transform.Find("Wheels/RearRight");

        WheelController wheelController1 = frontLeftWheel.GetComponent<WheelController>();
        WheelController wheelController2 = frontRightWheel.GetComponent<WheelController>();
        WheelController wheelController3 = rearLeftWheel.GetComponent<WheelController>();
        WheelController wheelController4 = rearRightWheel.GetComponent<WheelController>();

        wheelController1.enabled = false;
        wheelController2.enabled = false;
        wheelController3.enabled = false;
        wheelController4.enabled = false;

        // // for debugging...
        // // Check if the child object exists
        // if (frontLeftWheel != null)
        // {
        //     // Get the WheelController script component on the child object
        //     WheelController wheelController = frontLeftWheel.GetComponent<WheelController>();

        //     // Check if the component exists
        //     if (wheelController != null)
        //     {
        //         // Disable the script
        //         wheelController.enabled = false;
        //         Debug.Log("WheelController script disabled on FrontLeft wheel.");
        //     }
        //     else
        //     {
        //         Debug.LogWarning("WheelController component not found on FrontLeft wheel.");
        //     }
        // }
        // else
        // {
        //     Debug.LogWarning("FrontLeft wheel not found under Wheels.");
        // }
    }

}
