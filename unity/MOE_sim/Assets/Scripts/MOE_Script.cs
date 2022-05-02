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
    public GameObject Fx;
    public GameObject J0;
    public GameObject J1;
    public GameObject J2;
    public GameObject J3;

    [Header("Sliders")]
    public GameObject ShoulderSlider;
    public GameObject CounterweightSlider;
    public GameObject ForearmSlider;

    int shoulder_pos = 0;
    int counterweight_pos = 7;
    int forearm_pos = 7;

    int shoulder_pos_last = 0;
    int counterweight_pos_last = 7;
    int forearm_pos_last = 7;

    double[] qs = new double[4];

    int[] robot_params = new int[3];

     const string import_module = "virtual_moe";

    static IntPtr nativeLibraryPtr;
 
    delegate void start();
    delegate void stop();

    delegate void get_positions(double[] positions);
    delegate void update_mass_props(int forearm_pos, int counterweight_pos, double shoulder_pos);

    // bool printed = false;

    void Awake()
    {
        if (nativeLibraryPtr != IntPtr.Zero) return;
 
        nativeLibraryPtr = Native.LoadLibrary(import_module);
        if (nativeLibraryPtr == IntPtr.Zero)
        {
            Debug.LogError("Failed to load native library");
        }
    }

    // Use this for initialization
    void Start () {
        robot_params = new int[] {shoulder_pos, counterweight_pos, forearm_pos}; 
        Debug.Log("robot_params: " + robot_params[0] + " " + robot_params[1] + " " + robot_params[2]);
        Native.Invoke<update_mass_props>(nativeLibraryPtr, forearm_pos, counterweight_pos, shoulder_pos);
        Native.Invoke<start>(nativeLibraryPtr);        
	}

	// Update is called once per frame
	void Update () {
        // Dll.get_positions(qs);
        Native.Invoke<get_positions>(nativeLibraryPtr, qs);
        q0     = (float)qs[0];
        q1     = (float)qs[1];
        q2     = (float)qs[2];
        q3     = (float)qs[3];
        
        J0.transform.localEulerAngles   = new Vector3(0, 0, -q0*Mathf.Rad2Deg);
        J1.transform.localEulerAngles   = new Vector3(-q1*Mathf.Rad2Deg, 0, 0);
        J2.transform.localEulerAngles   = new Vector3(0, -q2*Mathf.Rad2Deg, 0);
        J3.transform.localEulerAngles   = new Vector3(0, 0, -q3*Mathf.Rad2Deg + 30);
    
        if (Input.GetKeyDown(KeyCode.R))
        {
            Native.Invoke<stop>(nativeLibraryPtr);
            Native.Invoke<start>(nativeLibraryPtr);
        }

        

        shoulder_pos = UpdateSlider(ShoulderSlider, "Shoulder Pos: ");
        counterweight_pos = UpdateSlider(CounterweightSlider, "Counterweight Pos: ");
        forearm_pos = UpdateSlider(ForearmSlider, "Forearm Pos: ");

        if (Input.GetMouseButtonUp(0)){
            if ((shoulder_pos      != shoulder_pos_last)      ||
                (counterweight_pos != counterweight_pos_last) ||
                (forearm_pos       != forearm_pos_last)){
                Debug.Log("Updating Mass Props");
                
                // robot_params = new int[] {shoulder_pos, counterweight_pos, forearm_pos}; 
                Native.Invoke<update_mass_props>(nativeLibraryPtr, forearm_pos, counterweight_pos, shoulder_pos);

                shoulder_pos_last = shoulder_pos;
                counterweight_pos_last = counterweight_pos;
                forearm_pos_last = forearm_pos;
            }
        }

    }

    void OnApplicationQuit() {
        Native.Invoke<stop>(nativeLibraryPtr);
        if (nativeLibraryPtr == IntPtr.Zero) return;
 
        Debug.Log(Native.FreeLibrary(nativeLibraryPtr)
                      ? "Native library successfully unloaded."
                      : "Native library could not be unloaded.");
    }

    int UpdateSlider(GameObject SliderObject, String title_text){
        int slider_value = (int)Mathf.Round(SliderObject.transform.Find("Slider").GetComponent<Slider>().value);
        if (title_text == "Shoulder Pos: ") slider_value = slider_value*15;
        SliderObject.transform.Find("Title").GetComponent<Text>().text = title_text + slider_value.ToString();
        return slider_value;
    }
}

