within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model Test
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  HeatFlowEpsilon heaFloEps(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    show_T=true)            annotation (Placement(transformation(extent={{0,-10},{20,10}})));
  Fluid.Sources.FixedBoundary           sinHea(
    redeclare package Medium = Medium,
    p=300000,
    T=313.15,
    nPorts=2) "Sink for heating medium"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={78,0})));
  Fluid.Sources.MassFlowSource_T           supHea(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=318.15,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,0})));
  HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273 + 20)
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=2000)
    annotation (Placement(transformation(extent={{-30,-2},{-10,18}})));
  Controls.OBC.CDL.Continuous.Sources.Ramp ram(
    height=2,
    duration=0.5,
    offset=-1,
    startTime=0.25) annotation (Placement(transformation(extent={{-96,-2},{-76,18}})));
  HeatingOrCooling_Epsilon heaOrCoo(
    redeclare package Medium = Medium,
    Q_flow_nominal=1000,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal=293.15) annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=0.5)
    annotation (Placement(transformation(extent={{-58,58},{-38,78}})));
  Fluid.Sources.MassFlowSource_T           supHea1(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    nPorts=1,
    T=318.15) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-50,42})));
equation
  connect(fixedTemperature.port, heaFloEps.heaPor)
    annotation (Line(points={{-20,-50},{10,-50},{10,-8}}, color={191,0,0}));
  connect(heaFloEps.UA, realExpression1.y) annotation (Line(points={{-2,8},{-9,8}}, color={0,0,127}));
  connect(supHea.m_flow_in, ram.y) annotation (Line(points={{-62,8},{-75,8}}, color={0,0,127}));
  connect(supHea.ports[1], heaFloEps.port_a)
    annotation (Line(points={{-40,0},{-20,0},{-20,0},{0,0}}, color={0,127,255}));
  connect(heaFloEps.port_b, sinHea.ports[1]) annotation (Line(points={{20,0},{44,0},{44,2},{68,2}}, color={0,127,255}));
  connect(heaOrCoo.port_b, sinHea.ports[2])
    annotation (Line(points={{10,60},{40,60},{40,-2},{68,-2}}, color={0,127,255}));
  connect(heaOrCoo.y, realExpression2.y) annotation (Line(points={{-12,68},{-37,68}}, color={0,0,127}));
  connect(fixedTemperature.port, heaOrCoo.heaPorLoa)
    annotation (Line(points={{-20,-50},{30,-50},{30,70},{0,70}}, color={191,0,0}));
  connect(ram.y, supHea1.m_flow_in) annotation (Line(points={{-75,8},{-68,8},{-68,50},{-62,50}}, color={0,0,127}));
  connect(supHea1.ports[1], heaOrCoo.port_a)
    annotation (Line(points={{-40,42},{-26,42},{-26,60},{-10,60}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Test;
