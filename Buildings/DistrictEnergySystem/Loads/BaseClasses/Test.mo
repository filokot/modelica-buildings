within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model Test
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  Fluid.Sources.FixedBoundary           sinHea(
    redeclare package Medium = Medium,
    nPorts=2,
    p=300000,
    T=313.15) "Sink for heating medium"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={78,0})));
  HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273.15 + 20)
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Controls.OBC.CDL.Continuous.Sources.Ramp ram(
    height=2,
    offset=-1,
    startTime=0.25,
    duration=100000)
                    annotation (Placement(transformation(extent={{-96,24},{-76,44}})));
  Fluid.Sources.MassFlowSource_T           supHea1(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    nPorts=1,
    T=318.15) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-52,60})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=273 - 5)
    annotation (Placement(transformation(extent={{-94,-64},{-74,-44}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=273 + 20)
    annotation (Placement(transformation(extent={{-94,-76},{-74,-56}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=1000)
    annotation (Placement(transformation(extent={{-94,-88},{-74,-68}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=800)
    annotation (Placement(transformation(extent={{-94,-102},{-74,-82}})));
  Modelica.Blocks.Sources.RealExpression realExpression6(y=heaOrCoo.m_flow_nominal)
    annotation (Placement(transformation(extent={{-100,58},{-80,78}})));
  HeatingOrCooling heaOrCoo(
    redeclare package Medium = Medium,
    Q_flow_nominal=1000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal=293.15) annotation (Placement(transformation(extent={{-10,40},{10,60}})));
  Modelica.Blocks.Sources.RealExpression realExpression1[1](y={heaOrCoo1.m_flow_nominal})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-44,-90})));
  Fluid.Sources.MassFlowSource_T           supHea2(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    nPorts=1,
    T=318.15) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-26,0})));
  HeatingOrCoolingMult heaOrCoo1(
    nLoa=1,
    Q_flowLoa_nominal={1000},
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal={293.15},
    reverseAction=false)   annotation (Placement(transformation(extent={{20,-76},{40,-60}})));
  Controls.OBC.CDL.Continuous.Product pro annotation (Placement(transformation(extent={{-62,18},{-42,38}})));
  Modelica.Blocks.Sources.RealExpression realExpression7[1](y={heaOrCoo1.Q_flow_nominal})
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-20,-90})));
equation
  connect(supHea1.ports[1], heaOrCoo.port_a) annotation (Line(points={{-42,60},{-34,60},{-34,50},{-10,50}},
                                                                                          color={0,127,255}));
  connect(heaOrCoo.port_b, sinHea.ports[1])
    annotation (Line(points={{10,50},{60,50},{60,2},{68,2}},   color={0,127,255}));
  connect(fixedTemperature.port, heaOrCoo.heaPorLoa)
    annotation (Line(points={{-20,-30},{34,-30},{34,60},{0,60}}, color={191,0,0}));
  connect(supHea1.m_flow_in, realExpression6.y) annotation (Line(points={{-64,68},{-79,68}}, color={0,0,127}));
  connect(fixedTemperature.port, heaOrCoo1.heaPorLoa[1])
    annotation (Line(points={{-20,-30},{30,-30},{30,-60}},        color={191,0,0}));
  connect(heaOrCoo1.port_b, sinHea.ports[2])
    annotation (Line(points={{40,-70},{54,-70},{54,-2},{68,-2}}, color={0,127,255}));
  connect(supHea2.ports[1], heaOrCoo1.port_a)
    annotation (Line(points={{-16,0},{0,0},{0,-70},{20,-70}}, color={0,127,255}));
  connect(ram.y, pro.u1) annotation (Line(points={{-75,34},{-64,34}},                 color={0,0,127}));
  connect(realExpression1[1].y, pro.u2)
    annotation (Line(points={{-44,-79},{-68,-79},{-68,22},{-64,22}}, color={0,0,127}));
  connect(pro.y, supHea2.m_flow_in) annotation (Line(points={{-41,28},{-40,28},{-40,8},{-38,8}}, color={0,0,127}));
  connect(realExpression7.y, heaOrCoo1.Q_flowLoaReq)
    annotation (Line(points={{-20,-79},{-20,-62},{18,-62}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Test;
