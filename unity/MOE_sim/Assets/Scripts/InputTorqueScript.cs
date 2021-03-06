using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using System;
using UnityEngine.UI;

using fts;

public class InputTorqueScript : MonoBehaviour
{
    public GameObject TorqueInputSingle;
    public GameObject TorqueInputMulti;
    public Slider TorqueSlider;
    public Slider EFESlider;
    public Slider FPSSlider;
    public Slider WFESlider;
    public Slider WRUSlider;
    public Text EFEText;
    public Text FPSText;
    public Text WFEText;
    public Text WRUText;

    [Header("Toggle")]
    public Toggle TorqueToggle;
    public Toggle InstructionsToggle;

    [Header("Dropdown")]
    public Dropdown DOFSelector;

    // torque input stuff
    float torque_input = 0;
    float input_reading_single = 0;
    float multiplier = (float)0.0;
    bool torque_on = false;
    bool torque_on_last = false;
    int selected_joint = 0;
    public static double[] input_torques = new double[4];
    int selected_joint_last = 0;
    float[] max_torques = {(float)3,(float)1,(float)0.5,(float)0.5};
    float[] input_reading_multi = {0,0,0,0};

    int dof_selection = 0;
    int dof_selection_last = 0;

    public void Start() {
        TorqueToggle.isOn = torque_on;
        input_torques[0] = (double)0;
        input_torques[1] = (double)0;
        input_torques[2] = (double)0;
        input_torques[3] = (double)0;
        TorqueSlider.minValue = (float)-1.0*max_torques[selected_joint];
        TorqueSlider.maxValue = max_torques[selected_joint];
        multiplier = max_torques[selected_joint];
        TorqueInputSingle.SetActive(false);
        TorqueInputMulti.SetActive(false);
    }
    public void Update () {
        // Update the torque input
        //torque_input = UpdateSlider(TorqueInputSingle, "Torque Input: ");
        torque_on = TorqueToggle.isOn;

        dof_selection = DOFSelector.value;
        // input_reading_single = UI.OnRightHorizontal();

        if ((torque_on) && (dof_selection == 1)) {
            input_reading_single = Input.GetAxis("LeftHorizontal");
            selected_joint = TorqueInputSingle.transform.Find("Dropdown").GetComponent<Dropdown>().value;
            // input_reading_single = PlayerActions.InputAction.RightHorizontal;
            // torque_input = input_reading_single*multiplier;
            torque_input = input_reading_single*max_torques[selected_joint];
            input_torques[selected_joint] = (double)torque_input;
            TorqueSlider.value = torque_input;
            TorqueInputSingle.transform.Find("Title").GetComponent<Text>().text = "Torque Input: " + myRound(torque_input,3).ToString() + " Nm"; 
        }

        if ((torque_on) && (dof_selection == 2)) {
            input_reading_multi[0] = Input.GetAxis("LeftHorizontal");
            input_reading_multi[1] = Input.GetAxis("LeftVertical");
            input_reading_multi[2] = Input.GetAxis("RightHorizontal");
            input_reading_multi[3] = Input.GetAxis("RightVertical");
            input_torques[0] = (double)(input_reading_multi[0]*max_torques[0]);
            input_torques[1] = (double)(input_reading_multi[1]*max_torques[1]);
            input_torques[2] = (double)(input_reading_multi[2]*max_torques[2]);
            input_torques[3] = (double)(input_reading_multi[3]*max_torques[3]);
            EFESlider.value = (float)input_torques[0];
            FPSSlider.value = (float)input_torques[1];
            WFESlider.value = (float)input_torques[2];
            WRUSlider.value = (float)input_torques[3];

            // WRUSlider.value = System.Round(WFESlider.value,3);

            EFEText.text = "EFE: " + myRound(EFESlider.value,3).ToString() + " Nm";
            FPSText.text = "FPS: " + myRound(FPSSlider.value,3).ToString() + " Nm";
            WFEText.text = "WFE: " + myRound(WFESlider.value,3).ToString() + " Nm";
            WRUText.text = "WRU: " + myRound(WRUSlider.value,3).ToString() + " Nm";
            // input_torques[0] = (float)(input_reading_multi[0]*multiplier[0]); 
        }
        if (selected_joint != selected_joint_last){
            input_torques[selected_joint_last] = (double)0.0;
            TorqueSlider.minValue = (float)-1.0*max_torques[selected_joint];
            TorqueSlider.maxValue = max_torques[selected_joint];
            multiplier = max_torques[selected_joint];
            selected_joint_last = selected_joint;
        }

        if (dof_selection != dof_selection_last){
            if (dof_selection_last == 1){
                TorqueInputSingle.SetActive(false);
            }
            if (dof_selection_last == 2) {
                TorqueInputMulti.SetActive(false);
            }
            if (dof_selection == 1){
                TorqueInputSingle.SetActive(true);
                multiplier = max_torques[selected_joint];
            }
            if (dof_selection == 2){
                TorqueInputMulti.SetActive(true);
            }
            dof_selection_last = dof_selection;
        }

        if (torque_on != torque_on_last){
            DOFSelector.value = 0;
            InstructionsToggle.isOn = false;
            torque_on_last = torque_on;
        }
    }

    float myRound(float inputValue, int decimalPoints){
        float internalMultiplier = (float)Mathf.Pow(10,decimalPoints);
        float myRoundedValue = (float)Mathf.Round(inputValue * internalMultiplier) / internalMultiplier;
        return myRoundedValue;
    }

}


