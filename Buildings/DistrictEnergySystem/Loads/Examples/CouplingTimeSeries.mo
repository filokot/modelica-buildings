within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingTimeSeries
  "Example illustrating the coupling of a time series building model to a fluid loop"
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
    annotation (Placement(transformation(extent={{110,-30},{90,-10}})));

  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.TimeSeriesBuilding
    bui(Q_flowHea_nominal={400}, Q_flowCoo_nominal={500})
    annotation (Placement(transformation(extent={{38,-40},{60,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-48,30})));
  Buildings.Fluid.Sources.FixedBoundary sinHea(
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for heating water"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={100,30})));
  Buildings.Fluid.Sources.FixedBoundary sinCoo(
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    p=300000,
    nPorts=1) "Sink for chilled water" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={100,-90})));
  Buildings.Fluid.Sources.MassFlowSource_T supCoo(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couCoo.T_a_nominal,
    nPorts=1)             "Supply for chilled water" annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-44,-90})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloHea(k=couHea.m_flow_nominal)
    annotation (Placement(transformation(extent={{-90,28},{-70,48}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloCoo(k=couCoo.m_flow_nominal)
    annotation (Placement(transformation(extent={{-88,-92},{-68,-72}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling couHea(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    Q_flow_nominal=bui.Q_flowHea_nominal[1],
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal=293.15) annotation (Placement(transformation(extent={{-20,40},{0,20}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling couCoo(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    Q_flow_nominal=bui.Q_flowCoo_nominal[1],
    T_a_nominal=280.15,
    T_b_nominal=285.15,
    TLoa_nominal=297.15) annotation (Placement(transformation(extent={{-20,-100},{0,-80}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai1(k=1/bui.Q_flowHea_nominal[1])
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-116,6})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai2(k=1/bui.Q_flowHea_nominal[1])
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-158,38})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDHea(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    reverseAction=false,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    k=1,
    Ti=10)  "PID controller for heating"
    annotation (Placement(transformation(extent={{-126,28},{-106,48}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai4(k=1/bui.Q_flowCoo_nominal[1])
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-110,-116})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai3(k=1/bui.Q_flowCoo_nominal[1])
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-150,-82})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDCoo(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    reverseAction=true,
    Ti=10)              "PID controller for cooling"
    annotation (Placement(transformation(extent={{-120,-92},{-100,-72}})));
equation
  connect(weaDat.weaBus, bui.weaBus) annotation (Line(
      points={{90,-20},{49.11,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(mFloHea.y, supHea.m_flow_in)
    annotation (Line(points={{-69,38},{-60,38}}, color={0,0,127}));
  connect(supCoo.m_flow_in, mFloCoo.y)
    annotation (Line(points={{-56,-82},{-67,-82}}, color={0,0,127}));

  connect(couCoo.port_b, sinCoo.ports[1]) annotation (Line(points={{0,-90},{90,-90}}, color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1]) annotation (Line(points={{0,30},{90,30}}, color={0,127,255}));
  connect(supHea.ports[1], couHea.port_a) annotation (Line(points={{-38,30},{-20,30}}, color={0,127,255}));
  connect(supCoo.ports[1], couCoo.port_a)
    annotation (Line(points={{-34,-90},{-28,-90},{-28,-90},{-20,-90}}, color={0,127,255}));
  connect(mFloHea.u, conPIDHea.y) annotation (Line(points={{-92,38},{-105,38}}, color={0,0,127}));
  connect(gai2.y, conPIDHea.u_s) annotation (Line(points={{-147,38},{-128,38}}, color={0,0,127}));
  connect(gai1.y, conPIDHea.u_m) annotation (Line(points={{-116,17},{-116,26}}, color={0,0,127}));
  connect(conPIDCoo.y, mFloCoo.u) annotation (Line(points={{-99,-82},{-90,-82}}, color={0,0,127}));
  connect(gai3.y, conPIDCoo.u_s) annotation (Line(points={{-139,-82},{-122,-82}}, color={0,0,127}));
  connect(gai4.y, conPIDCoo.u_m) annotation (Line(points={{-110,-105},{-110,-94}}, color={0,0,127}));
  connect(bui.Q_flowHeaAct[1], gai1.u)
    annotation (Line(points={{61.1,-21},{66,-21},{66,-16},{-116,-16},{-116,-6}}, color={0,0,127}));
  connect(bui.Q_flowHeaReq[1], gai2.u)
    annotation (Line(points={{61.1,-27},{68,-27},{68,60},{-180,60},{-180,38},{-170,38}}, color={0,0,127}));
  connect(bui.Q_flowCooAct[1], gai4.u)
    annotation (Line(points={{61.1,-39},{66,-39},{66,-140},{-110,-140},{-110,-128}}, color={0,0,127}));
  connect(bui.Q_flowCooReq[1], gai3.u)
    annotation (Line(points={{61.1,-33},{70,-33},{70,-150},{-180,-150},{-180,-82},{-162,-82}}, color={0,0,127}));
  connect(couHea.heaPorLoa[1], bui.heaPorHea[1])
    annotation (Line(points={{-10,20},{-10,-23},{38,-23}}, color={191,0,0}));
  connect(couCoo.heaPorLoa[1], bui.heaPorCoo[1])
    annotation (Line(points={{-10,-80},{-10,-37},{38,-37}}, color={191,0,0}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-200,-160},{120,80}})),
    __Dymola_Commands(file=
          "Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingTimeSeries.mos"
        "Simulate and plot"));
end CouplingTimeSeries;
