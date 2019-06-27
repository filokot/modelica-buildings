within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingRC "Example illustrating the coupling of a RC building model to a fluid loop"
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

  Buildings.DistrictEnergySystem.Loads.Examples.BaseClasses.RCOneElementBuilding
    bui(Q_flowCoo_nominal={2500}, Q_flowHea_nominal={500})
    annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Buildings.Fluid.Sources.MassFlowSource_T supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-32,30})));
  Buildings.Fluid.Sources.FixedBoundary sinHea(
    redeclare package Medium = Medium,
    T=couHea.T_a_nominal,
    nPorts=1,
    p=300000) "Sink for heating water"
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
        origin={-30,-90})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloHea(k=couHea.m_flow_nominal)
    annotation (Placement(transformation(extent={{-80,28},{-60,48}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain mFloCoo(k=couCoo.m_flow_nominal)
    annotation (Placement(transformation(extent={{-80,-92},{-60,-72}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai2(k=1/sum(bui.Q_flowHea_nominal))
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-150,38})));
  Buildings.Controls.OBC.CDL.Continuous.LimPID conPIDHea(
    yMax=1,
    controllerType=Buildings.Controls.OBC.CDL.Types.SimpleController.PI,
    reverseAction=false,
    yMin=0,
    initType=Buildings.Controls.OBC.CDL.Types.Init.InitialOutput,
    k=1,
    Ti=120) "PID controller for heating"
    annotation (Placement(transformation(extent={{-120,28},{-100,48}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai1(k=1/sum(bui.Q_flowHea_nominal))
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-110,10})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai3(k=1/sum(bui.Q_flowCoo_nominal))
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
    Ti=120)             "PID controller for cooling"
    annotation (Placement(transformation(extent={{-120,-92},{-100,-72}})));
  Buildings.Controls.OBC.CDL.Continuous.Gain gai4(k=1/sum(bui.Q_flowCoo_nominal))
    annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-110,-110})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling couHea(
    redeclare package Medium = Medium,
    Q_flow_nominal=bui.Q_flowHea_nominal[1],
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal=293.15) annotation (Placement(transformation(extent={{0,40},{20,20}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCooling couCoo(
    redeclare package Medium = Medium,
    Q_flow_nominal=bui.Q_flowCoo_nominal[1],
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=280.15,
    T_b_nominal=285.15,
    TLoa_nominal=297.15) annotation (Placement(transformation(extent={{0,-100},{20,-80}})));
equation
  connect(weaDat.weaBus, bui.weaBus) annotation (Line(
      points={{90,-20},{50.1,-20}},
      color={255,204,51},
      thickness=0.5));
  connect(mFloHea.y, supHea.m_flow_in)
    annotation (Line(points={{-59,38},{-44,38}}, color={0,0,127}));
  connect(supCoo.m_flow_in, mFloCoo.y)
    annotation (Line(points={{-42,-82},{-59,-82}}, color={0,0,127}));

  connect(supHea.ports[1], couHea.port_a) annotation (Line(points={{-22,30},{0,30}},   color={0,127,255}));
  connect(couHea.port_b, sinHea.ports[1]) annotation (Line(points={{20,30},{90,30}},color={0,127,255}));
  connect(supCoo.ports[1], couCoo.port_a) annotation (Line(points={{-20,-90},{0,-90}},   color={0,127,255}));
  connect(couCoo.port_b, sinCoo.ports[1]) annotation (Line(points={{20,-90},{90,-90}},color={0,127,255}));
  connect(conPIDHea.y, mFloHea.u) annotation (Line(points={{-99,38},{-82,38}},  color={0,0,127}));
  connect(gai2.y, conPIDHea.u_s) annotation (Line(points={{-139,38},{-122,38}}, color={0,0,127}));
  connect(gai1.y, conPIDHea.u_m) annotation (Line(points={{-110,21},{-110,26}}, color={0,0,127}));
  connect(gai4.y, conPIDCoo.u_m) annotation (Line(points={{-110,-99},{-110,-94}},  color={0,0,127}));
  connect(gai3.y, conPIDCoo.u_s) annotation (Line(points={{-139,-82},{-122,-82}}, color={0,0,127}));
  connect(conPIDCoo.y, mFloCoo.u) annotation (Line(points={{-99,-82},{-82,-82}}, color={0,0,127}));
  connect(couCoo.heaPorLoa, bui.heaPorCoo[1]) annotation (Line(points={{10,-80},{10,-37},{40,-37}}, color={191,0,0}));
  connect(couHea.heaPorLoa, bui.heaPorHea[1]) annotation (Line(points={{10,20},{10,-23},{40,-23}}, color={191,0,0}));
  connect(bui.Q_flowHeaAct[1], gai1.u)
    annotation (Line(points={{61,-21},{66,-21},{66,-12},{-110,-12},{-110,-2}}, color={0,0,127}));
  connect(bui.Q_flowCooAct[1], gai4.u)
    annotation (Line(points={{61,-39},{66,-39},{66,-140},{-110,-140},{-110,-122}}, color={0,0,127}));
  connect(bui.Q_flowHeaReq[1], gai2.u)
    annotation (Line(points={{61,-27},{80,-27},{80,60},{-180,60},{-180,38},{-162,38}}, color={0,0,127}));
  connect(bui.Q_flowCooReq[1], gai3.u)
    annotation (Line(points={{61,-33},{80,-33},{80,-160},{-180,-160},{-180,-82},{-162,-82}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-220,-180},{160,100}})),
    __Dymola_Commands(file="Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingRC.mos"
        "Simulate and plot"));
end CouplingRC;
