within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingRCOneElementNoPort
  "Example illustrating the coupling of a RC building model and a ETS model"
  import Buildings;
  extends Modelica.Icons.Example;
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                            weaDat(
    calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
    computeWetBulbTemperature=false,
    filNam=Modelica.Utilities.Files.loadResource(
        "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
    "Weather data reader"
    annotation (Placement(transformation(extent={{40,0},{60,20}})));

  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.RCOneElementBuildingNoPort
    bui(Q_flowHea_nominal=500, Q_flowCoo_nominal=1000)
    annotation (Placement(transformation(extent={{38,-40},{60,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-48,50})));
  Buildings.Fluid.Sources.FixedBoundary sinHea(
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for heating medium"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={96,50})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conFloHea(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    yMin=0,
    reverseAction=false,
    k=1,
    Ti=120)
           "PI controller for heating"
    annotation (Placement(transformation(extent={{-126,48},{-106,68}})));
  Buildings.Fluid.Sources.FixedBoundary sinCoo(
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for cooling medium" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={94,-90})));
  Buildings.Fluid.Sources.MassFlowSource_T supCoo(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    nPorts=1)             "Supply for chilled water" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-44,-90})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conFloCoo(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    yMin=0,
    reverseAction=true,
    k=1,
    Ti=120)
           "PI controller for cooling"
    annotation (Placement(transformation(extent={{-120,-92},{-100,-72}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingNoPort
    couHea(
    Q_flow_nominal=bui.Q_flowHea_nominal,
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TInd_nominal=293.15)
    annotation (Placement(transformation(extent={{-20,40},{0,60}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingNoPort
    couCoo(
    Q_flow_nominal=bui.Q_flowCoo_nominal,
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=280.15,
    T_b_nominal=285.15,
    TInd_nominal=297.15)
    annotation (Placement(transformation(extent={{-20,-100},{0,-80}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloHea(k=couHea.m_flow_nominal)
    annotation (Placement(transformation(extent={{-90,48},{-70,68}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai1(k=1/bui.Q_flowHea_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-116,30})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai2(k=1/bui.Q_flowHea_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-144,58})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai3(k=1/bui.Q_flowCoo_nominal)
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-148,-82})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai4(k=1/bui.Q_flowCoo_nominal)
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-110,-112})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloCoo(k=couCoo.m_flow_nominal)
    annotation (Placement(transformation(extent={{-88,-92},{-68,-72}})));
equation
  connect(weaDat.weaBus, bui.weaBus) annotation (Line(
      points={{60,10},{60,-20},{49.11,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(supHea.ports[1], couHea.port_a)
    annotation (Line(points={{-38,50},{-20,50}}, color={0,127,255}));
  connect(supCoo.ports[1], couCoo.port_a)
    annotation (Line(points={{-34,-90},{-20,-90}}, color={0,127,255}));
  connect(couCoo.Q_flowAva, bui.Q_flowCooAva) annotation (Line(points={{1,-85},{
          20.5,-85},{20.5,-36},{35.8,-36}},
                                         color={0,0,127}));
  connect(couCoo.port_b, sinCoo.ports[1])
    annotation (Line(points={{0,-90},{84,-90}},  color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1])
    annotation (Line(points={{0,50},{86,50}},  color={0,127,255}));
  connect(couHea.Q_flowAva, bui.Q_flowHeaAva) annotation (Line(points={{1,55},{20,
          55},{20,-24},{35.8,-24}},   color={0,0,127}));
  connect(conFloHea.y, mFloHea.u)
    annotation (Line(points={{-105,58},{-92,58}}, color={0,0,127}));
  connect(mFloHea.y, supHea.m_flow_in)
    annotation (Line(points={{-69,58},{-60,58}}, color={0,0,127}));
  connect(conFloHea.u_m, gai1.y)
    annotation (Line(points={{-116,46},{-116,41}}, color={0,0,127}));
  connect(conFloHea.u_s, gai2.y)
    annotation (Line(points={{-128,58},{-133,58}}, color={0,0,127}));
  connect(bui.Q_flowHeaReq, gai2.u) annotation (Line(points={{61.1,-26},{70,-26},
          {70,76},{-164,76},{-164,58},{-156,58}}, color={0,0,127}));
  connect(couHea.Q_flowAva, gai1.u) annotation (Line(points={{1,55},{20,55},{20,
          10},{-116,10},{-116,18}},      color={0,0,127}));
  connect(conFloCoo.u_s, gai3.y)
    annotation (Line(points={{-122,-82},{-137,-82}}, color={0,0,127}));
  connect(conFloCoo.u_m, gai4.y)
    annotation (Line(points={{-110,-94},{-110,-101}},color={0,0,127}));
  connect(couCoo.Q_flowAva, gai4.u) annotation (Line(points={{1,-85},{20,-85},{20,
          -132},{-110,-132},{-110,-124}},     color={0,0,127}));
  connect(bui.Q_flowCooReq, gai3.u) annotation (Line(points={{61.1,-34},{68,-34},
          {68,-64},{-166,-64},{-166,-82},{-160,-82}},        color={0,0,127}));
  connect(conFloCoo.y, mFloCoo.u)
    annotation (Line(points={{-99,-82},{-90,-82}},  color={0,0,127}));
  connect(supCoo.m_flow_in, mFloCoo.y)
    annotation (Line(points={{-56,-82},{-67,-82}}, color={0,0,127}));
  connect(bui.TInd, couHea.TInd) annotation (Line(points={{61.1,-30},{74,-30},{74,
          62},{-26,62},{-26,58},{-22,58}}, color={0,0,127}));
  connect(bui.TInd, couCoo.TInd) annotation (Line(points={{61.1,-30},{74,-30},{74,
          -76},{-26,-76},{-26,-82},{-22,-82}}, color={0,0,127}));
  connect(bui.Q_flowCooAct, couCoo.Q_flowAct) annotation (Line(points={{61.1,-38},
          {64,-38},{64,-108},{-26,-108},{-26,-98},{-22,-98}},
                                                            color={0,0,127}));
  connect(bui.Q_flowHeaAct, couHea.Q_flowAct) annotation (Line(points={{61.1,-22},
          {66,-22},{66,36},{-26,36},{-26,42},{-22,42}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-200,-160},{120,100}})),
    __Dymola_Commands(file=
          "Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingRCOneElementNoPort.mos"
        "Simulate and plot"));
end CouplingRCOneElementNoPort;
