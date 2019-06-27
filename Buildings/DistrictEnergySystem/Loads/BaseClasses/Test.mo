within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model Test
  package Medium = Buildings.Media.Water "Fluid in the pipes";
  Fluid.Sources.FixedBoundary           sinHea(
    redeclare package Medium = Medium,
    nPorts=1,
    p=300000,
    T=313.15) "Sink for heating medium"
                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={78,0})));
  HeatTransfer.Sources.FixedTemperature fixedTemperature(T=273 + 20)
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Controls.OBC.CDL.Continuous.Sources.Ramp ram(
    height=2,
    duration=0.5,
    offset=-1,
    startTime=0.25) annotation (Placement(transformation(extent={{-96,-2},{-76,18}})));
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
  Modelica.Blocks.Sources.RealExpression realExpression6(y=400)
    annotation (Placement(transformation(extent={{-40,74},{-20,94}})));
equation
  connect(supHea1.ports[1], heaOrCoo.port_a) annotation (Line(points={{-42,60},{-10,60}}, color={0,127,255}));
  connect(heaOrCoo.port_b, sinHea.ports[1])
    annotation (Line(points={{10,60},{60,60},{60,0},{68,0}},   color={0,127,255}));
  connect(fixedTemperature.port, heaOrCoo.heaPorLoa)
    annotation (Line(points={{-20,-30},{34,-30},{34,70},{0,70}}, color={191,0,0}));
  connect(realExpression6.y, heaOrCoo.Q_flow)
    annotation (Line(points={{-19,84},{-16,84},{-16,64},{-12,64}}, color={0,0,127}));
  connect(heaOrCoo.m_flowReq, supHea1.m_flow_in)
    annotation (Line(points={{11,64},{18,64},{18,68},{-64,68}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Test;
