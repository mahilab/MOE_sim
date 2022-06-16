using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;
using System;
using UnityEngine.UI;

using fts;

public class MOE_Script : MonoBehaviour {

    [Header("Positions")]
    public float q0;
    public float q1;
    public float q2;
    public float q3;

    [Header("Inertial Frame References")]
    public GameObject Fx_PostShoulder;
    public GameObject J0_main;
    public GameObject J0_slider;
    public GameObject J0_counterweight;
    public GameObject J1;
    public GameObject J2;
    public GameObject J3;
    // public GameObject UI;

    [Header("Sliders")]
    public GameObject ShoulderSlider;
    public GameObject CounterweightSlider;
    public GameObject ForearmSlider;

    public GameObject ArmWarning;


    int shoulder_pos = 0;
    int counterweight_pos = 7;
    int slider_pos = 7;


    int shoulder_pos_last = 0;
    int counterweight_pos_last = 7;
    int slider_pos_last = 7;

    double[] qs = new double[4];
    double[] input_torques = {0.0,0.0,0.0,0.0};
    float MassPropRefreshTime = 0.25f;
    float CurrentTime = 0.25f;

    const string import_module = "virtual_moe";

    static IntPtr nativeLibraryPtr;
 
    delegate void start();
    delegate void stop();

    delegate void get_positions(double[] positions);
    delegate void send_torques(double[] torques);
    delegate void update_mass_props(int slider_pos, int counterweight_pos, int shoulder_pos);
    delegate void add_arm_model();

    // opens the virtual_moe.dll provided
    void Awake()
    {
        if (nativeLibraryPtr != IntPtr.Zero) return;
 
        nativeLibraryPtr = Native.LoadLibrary(import_module);
        if (nativeLibraryPtr == IntPtr.Zero)
        {
            Debug.LogError("Failed to load native library");
        }
    }

    void Start () {
        // update the mass properties with the initial conditions
        Native.Invoke<update_mass_props>(nativeLibraryPtr, slider_pos, counterweight_pos, shoulder_pos);
        // start the simulation
        Native.Invoke<start>(nativeLibraryPtr);        
        // update the visualization with the proper visualization based on user parameters
        UpdateParameterVisuals();
        // Turn off torque input by default

	}

	// Update is called once per frame
	void Update () {
        // get positions from the simulation
        Native.Invoke<get_positions>(nativeLibraryPtr, qs);
        q0     = (float)qs[0];
        q1     = (float)qs[1];
        q2     = (float)qs[2];
        q3     = (float)qs[3];
        
        // update the visualization based on the joint positions
        J0_main.transform.localEulerAngles   = new Vector3(0, 0, -q0*Mathf.Rad2Deg);
        J1.transform.localEulerAngles        = new Vector3(-q1*Mathf.Rad2Deg, 0, 0);
        J2.transform.localEulerAngles        = new Vector3(0, -q2*Mathf.Rad2Deg, 0);
        J3.transform.localEulerAngles        = new Vector3(0, 0, -q3*Mathf.Rad2Deg);
    
        // if the user hits R, reset the simulation
        if (Input.GetKeyDown(KeyCode.R))
        {
            Native.Invoke<stop>(nativeLibraryPtr);
            Native.Invoke<start>(nativeLibraryPtr);
        }

        CurrentTime -= Time.deltaTime;

        // if update the text above the sliders based on the slider positions
        shoulder_pos = UpdateSlider(ShoulderSlider, "Shoulder Pos: ");
        // ShoulderSlider.transform.Find("Slider").
        counterweight_pos = UpdateSlider(CounterweightSlider, "Counterweight Pos: ");
        slider_pos = UpdateSlider(ForearmSlider, "Forearm Pos: ");
        


        // input_torques = InputTorqueScript.pass_torques();
        input_torques = InputTorqueScript.input_torques;
        Native.Invoke<send_torques>(nativeLibraryPtr, input_torques);
        // send_torques(input_torques);
        // SliderObject.transform.Find("Title").GetComponent<Text>().text = title_text + torque_input.ToString();

        // update visualization of user params based on the slider positions
        UpdateParameterVisuals();

        // if anything has changed, updated the moe model
        if (((shoulder_pos      != shoulder_pos_last)      ||
            (counterweight_pos != counterweight_pos_last) ||
            (slider_pos        != slider_pos_last)) 
            && (CurrentTime <= 0)){
            Debug.Log(CurrentTime);
            
            Native.Invoke<update_mass_props>(nativeLibraryPtr, slider_pos, counterweight_pos, shoulder_pos);

            shoulder_pos_last = shoulder_pos;
            counterweight_pos_last = counterweight_pos;
            slider_pos_last = slider_pos;

            CurrentTime = MassPropRefreshTime;
        }



    }

    void UpdateParameterVisuals(){
        Fx_PostShoulder.transform.localEulerAngles = new Vector3( -1*shoulder_pos*15.0f, 0, 0);
        J0_slider.transform.localPosition = new Vector3(0.005f*(slider_pos-3), 0, 0);
        J0_counterweight.transform.localPosition = new Vector3( 0.0127f*(counterweight_pos-4), 0, 0);
    }

    // close the sim
    void OnApplicationQuit() {
        Native.Invoke<stop>(nativeLibraryPtr);
        if (nativeLibraryPtr == IntPtr.Zero) return;
 
        Debug.Log(Native.FreeLibrary(nativeLibraryPtr)
                      ? "Native library successfully unloaded."
                      : "Native library could not be unloaded.");
    }

    int UpdateSlider(GameObject SliderObject, String title_text){
        // pull the slider value
        int slider_value = (int)Mathf.Round(SliderObject.transform.Find("Slider").GetComponent<Slider>().value);
        // update the text of the slider to match the value
        SliderObject.transform.Find("Title").GetComponent<Text>().text = title_text + slider_value.ToString();
        return slider_value;
    }


    public void dll_add_arm_model(){
        Native.Invoke<add_arm_model>(nativeLibraryPtr);

        ShoulderSlider.transform.Find("Slider").GetComponent<Slider>().enabled = false;
        ForearmSlider.transform.Find("Slider").GetComponent<Slider>().enabled = false;
        CounterweightSlider.transform.Find("Slider").GetComponent<Slider>().enabled = false;
        ArmWarning.SetActive(true);
    }


}

