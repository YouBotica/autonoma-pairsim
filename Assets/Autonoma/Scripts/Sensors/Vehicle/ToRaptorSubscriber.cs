/* 
Copyright 2023 Autonoma, Inc.

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
using ROS2;
using AWSIM;
using autonoma_msgs.msg;
using std_msgs.msg;


namespace Autonoma
{
public class ToRaptorSubscriber : MonoBehaviour
{
    public string toRaptorTopic = "/to_raptor";

    public string ResetSimulatorTopic = "/simulator/reset";
    public QoSSettings qosSettings = new QoSSettings();
    public RaptorSM sm;
    ISubscription<ToRaptor> toRaptorSubscriber;
    CanSubscriber canToRaptorSubscriber;

    ISubscription<Bool> resetSimulatorSubscriber;


    void Start()
    {
        var qos = qosSettings.GetQoSProfile();

        Debug.Log("Creating subscription to " + toRaptorTopic);

        toRaptorSubscriber = SimulatorROS2Node.CreateSubscription<ToRaptor>(toRaptorTopic, msg =>
            {
                UpdateToRaptor(msg);
            }, qos);

        canToRaptorSubscriber = new CanSubscriber("ct_report", qosSettings, data_values => {
            sm.current_ct = (int)data_values[2];
        });

        resetSimulatorSubscriber = SimulatorROS2Node.CreateSubscription<Bool>(ResetSimulatorTopic, msg =>
            {
                Debug.Log("Got reset message: " + msg.Data);
                ResetSimulation(msg);
            }, qos);
    }
    void OnDestroy()
    {
        SimulatorROS2Node.RemoveSubscription<ToRaptor>(toRaptorSubscriber);
        SimulatorROS2Node.RemoveSubscription<Bool>(resetSimulatorSubscriber);
    }
    void UpdateToRaptor(ToRaptor msg)
    {
        sm.current_ct = msg.Ct_state;
    }

    void ResetSimulation(Bool msg)
    {
        Debug.Log("Got reset message: " + msg.Data);
        GameManager.Instance.ChangeStateTo(GameManager.SimulationState.RESTART);
        // UIManager = FindObjectOfType<SimUIManager>();
        // UIManager.Instance.ChangeStateTo(GameManager.SimulationState.RESTART);
    }
}
}