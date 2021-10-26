using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;

namespace ConsoleApp1
{
    public class SWMassProperties
    {
        public double Pcx { get; set; }
        public double Pcy { get; set; }
        public double Pcz { get; set; }
        public double m { get; set; }
        public double Icxx { get; set; }
        public double Icyy { get; set; }
        public double Iczz { get; set; }
        public double Icxy { get; set; }
        public double Icxz { get; set; }
        public double Icyz { get; set; }
    }

    class Application
    {
        SldWorks.ModelDoc2 doc;
        static int fileerror;
        static int filewarning;
        [STAThread]
        static void Main(string[] args)
        {
			Console.WriteLine("here1");
			ModelDoc2 swModel = default(ModelDoc2);
            ModelDocExtension swModelExt = default(ModelDocExtension);
            SelectionMgr swSelMgr = default(SelectionMgr);
            Component2 swComp = default(Component2);
            int nStatus = 0;
            double[] vMassProp = null;
            int i = 0;
            int nbrSelections = 0;

			//string[,] = dof_component_names = new string[,] {{"exo_forearm-1", "OW_subasm1-1"},
			//												 {""},
			Console.WriteLine("here1");

            List<List<string>> dof_component_names = new List<List<string>>() {new List<string>(){"exo_forearm-1",  "OW_subasm1-1"},
                                                                               new List<string>(){"OWL_subasm2-1", "OWL_subasm3-1",  "OW_subasm5-1"},
                                                                               new List<string>(){ "OW_subasm6-1", "OWL_subasm7-1", "OWL_subasm8-1"},
                                                                               new List<string>(){"OW_subasm10-1", "OW_subasm11-1", "OW_subasm12-1"}};

            //int joint_num = 1;

            Console.WriteLine("here2");

            SldWorks.SldWorks swApp;
       
            swApp = new SldWorks.SldWorks();

            swApp.Visible = true;

            swApp.OpenDoc6("C://Users//nbd2//Box//MAHI_OW_w_Elbow//MOE_forNathan//MOE//MOE_fullasm.SLDASM", (int)swDocumentTypes_e.swDocASSEMBLY, (int)swOpenDocOptions_e.swOpenDocOptions_Silent, "", ref fileerror, ref filewarning);

            swModel = (ModelDoc2)swApp.ActiveDoc;
            swModelExt = swModel.Extension;
            swSelMgr = (SelectionMgr)swModel.SelectionManager;

            for (int joint_num = 0; joint_num < 4; joint_num++)
            {
                foreach (var component_name in dof_component_names[joint_num])
                {
                    swModelExt.SelectByID2(component_name, "COMPONENT", 0, 0, 0, true, 0, null, 0);
                }
                //swModelExt.SelectByID2("OW_subasm1-1", "COMPONENT", 0, 0, 0, true, 0, null, 0);

                MassProperty2 MyMassProp = (MassProperty2)swModelExt.CreateMassProperty2();
                var currCoordSystem = swModelExt.GetCoordinateSystemTransformByName("CS_" + joint_num);
                MyMassProp.SetCoordinateSystem(currCoordSystem);
				MyMassProp.IncludeHiddenBodiesOrComponents = false;
                MyMassProp.Recalculate();
                double[] com = (double[])MyMassProp.CenterOfMass;
                double[] moi = (double[])MyMassProp.GetMomentOfInertia(1);

                Console.WriteLine("Center of mass for joint " + joint_num);
                Console.WriteLine("  X-coordinate = " + com[0]);
                Console.WriteLine("  Y-coordinate = " + com[1]);
                Console.WriteLine("  Z-coordinate = " + com[2]);
                Console.WriteLine("Volume = " + MyMassProp.Volume);
                Console.WriteLine("Surface area = " + MyMassProp.SurfaceArea);
                Console.WriteLine("Mass = " + MyMassProp.Mass);
                Console.WriteLine("Density = " + MyMassProp.Density);
                Console.WriteLine("Moments of inertia taken at the center of mass and aligned with the output coordinate system:");
                Console.WriteLine("  Lxx = " + moi[0]);
                Console.WriteLine("  Lxy = " + moi[1]);
                Console.WriteLine("  Lxz = " + moi[2]);
                Console.WriteLine("  Lyx = " + moi[3]);
                Console.WriteLine("  Lyy = " + moi[4]);
                Console.WriteLine("  Lyz = " + moi[5]);
                Console.WriteLine("  Lzx = " + moi[6]);
                Console.WriteLine("  Lzy = " + moi[7]);
                Console.WriteLine("  Lzz = " + moi[8]);

                //Console.WriteLine("JSON OUTPUT");

                var sw_mass_properties = new SWMassProperties
                {
                    Pcx = com[0],
                    Pcy = com[1],
                    Pcz = com[2],
                    m = MyMassProp.Mass,
                    Icxx = moi[0],
                    Icyy = moi[4],
                    Iczz = moi[8],
                    Icxy = moi[1],
                    Icxz = moi[2],
                    Icyz = moi[5]
                };

                string jsonString = JsonSerializer.Serialize(sw_mass_properties);

                string fileName = "C://Git//MOE_sim//util//mass_properties//Joint" + joint_num + "_mass_properties.json";
                File.WriteAllText(fileName, jsonString);

                Console.ReadLine();
                foreach (var component_name in dof_component_names[joint_num])
                {
                    swSelMgr.DeSelect2(1, -1);
                }
            }

            

            swApp.ExitApp();
            swApp = null;
        }

    }
}
