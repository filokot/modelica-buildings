within Buildings.DistrictEnergySystem.Loads.Examples;
model CouplingRCSplit "Example illustrating the coupling of a RC building model to a fluid loop"
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
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingSplit couHea(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    Q_flowLoa_nominal=bui.Q_flowHea_nominal,
    TLoa_nominal=fill(273.15 + 20, size(bui.Q_flowHea_nominal, 1)),
    nLoa=1) annotation (Placement(transformation(extent={{0,40},{20,20}})));
  Buildings.DistrictEnergySystem.Loads.BaseClasses.HeatingOrCoolingSplit couCoo(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=280.15,
    T_b_nominal=285.15,
    Q_flowLoa_nominal=bui.Q_flowCoo_nominal,
    TLoa_nominal=fill(24 + 273.15, size(bui.Q_flowCoo_nominal, 1)),
    nLoa=1) annotation (Placement(transformation(extent={{0,-100},{20,-80}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum(nin=1)
    annotation (Placement(transformation(extent={{-200,28},{-180,48}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum3(nin=1)
    annotation (Placement(transformation(extent={{-200,-92},{-180,-72}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum1(nin=1)
    annotation (Placement(transformation(extent={{-60,-16},{-80,4}})));
  Buildings.Controls.OBC.CDL.Continuous.MultiSum mulSum2(nin=1)
    annotation (Placement(transformation(extent={{-60,-150},{-80,-130}})));
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
  connect(mulSum1.y, gai1.u) annotation (Line(points={{-81,-6},{-110,-6},{-110,-2}}, color={0,0,127}));
  connect(mulSum2.y, gai4.u) annotation (Line(points={{-81,-140},{-110,-140},{-110,-122}}, color={0,0,127}));
  connect(mulSum3.y, gai3.u) annotation (Line(points={{-179,-82},{-162,-82}}, color={0,0,127}));
  connect(bui.heaPorHea, couHea.heaPorLoa[1:1]) annotation (Line(points={{40,-23},{10,-23},{10,20}}, color={191,0,0}));
  connect(bui.heaPorCoo, couCoo.heaPorLoa[1:1]) annotation (Line(points={{40,-37},{10,-37},{10,-80}}, color={191,0,0}));
  connect(mulSum.y, gai2.u) annotation (Line(points={{-179,38},{-162,38}}, color={0,0,127}));
  connect(bui.Q_flowCooReq, mulSum3.u[1:1])
    annotation (Line(points={{61,-33},{80,-33},{80,-60},{-220,-60},{-220,-82},{-202,-82}}, color={0,0,127}));
  connect(bui.Q_flowCooAct, mulSum2.u[1:1])
    annotation (Line(points={{61,-39},{70,-39},{70,-140},{-58,-140}}, color={0,0,127}));
  connect(bui.Q_flowHeaAct, mulSum1.u[1:1])
    annotation (Line(points={{61,-21},{68,-21},{68,-6},{-58,-6}}, color={0,0,127}));
  connect(bui.Q_flowHeaReq, mulSum.u[1:1])
    annotation (Line(points={{61,-27},{80,-27},{80,60},{-220,60},{-220,38},{-202,38}}, color={0,0,127}));
  connect(bui.Q_flowHeaReq, couHea.Q_flow[1:1])
    annotation (Line(points={{61,-27},{80,-27},{80,8},{-10,8},{-10,26},{-2,26}}, color={0,0,127}));
  connect(bui.Q_flowCooReq, couCoo.Q_flow[1:1])
    annotation (Line(points={{61,-33},{80,-33},{80,-74},{-10,-74},{-10,-86},{-2,-86}}, color={0,0,127}));
  annotation (Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-220,-180},{160,100}})),
    __Dymola_Commands(file="Resources/Scripts/Dymola/DistrictEnergySystem/Loads/Examples/CouplingRC.mos"
        "Simulate and plot"));
end CouplingRCSplit;
