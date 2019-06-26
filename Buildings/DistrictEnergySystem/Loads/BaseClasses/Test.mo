within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model Test
  package Medium = Buildings.Media.Water "Fluid in the pipes";
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
    annotation (Placement(transformation(extent={{-40,-40},{-20,-20}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=2000)
    annotation (Placement(transformation(extent={{-30,-2},{-10,18}})));
  Controls.OBC.CDL.Continuous.Sources.Ramp ram(
    height=2,
    duration=0.5,
    offset=-1,
    startTime=0.25) annotation (Placement(transformation(extent={{-96,-2},{-76,18}})));
  HeatFlowEffectiveness heaFloEps(
    redeclare package Medium = Medium,
    m_flow_nominal=1,
    show_T=true) annotation (Placement(transformation(extent={{6,-10},{26,10}})));
  HeatingOrCooling heaOrCoo(
    redeclare package Medium = Medium,
    Q_flow_nominal=1000,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    T_a_nominal=318.15,
    T_b_nominal=313.15,
    TLoa_nominal=293.15) annotation (Placement(transformation(extent={{-10,50},{10,70}})));
  Fluid.Sources.MassFlowSource_T           supHea1(
    use_m_flow_in=true,
    redeclare package Medium = Medium,
    T=318.15,
    nPorts=1) "Supply for heating water"          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-52,60})));
  SimplifiedBuildingODE simplifiedBuildingODE(
    Q_flowHea_nominal=1000,
    TOutHea_nominal=268.15,
    TIndHea_nominal=293.15) annotation (Placement(transformation(extent={{-20,-80},{0,-60}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=273 - 5)
    annotation (Placement(transformation(extent={{-94,-64},{-74,-44}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=273 + 20)
    annotation (Placement(transformation(extent={{-94,-76},{-74,-56}})));
  Modelica.Blocks.Sources.RealExpression realExpression4(y=1000)
    annotation (Placement(transformation(extent={{-94,-88},{-74,-68}})));
  Modelica.Blocks.Sources.RealExpression realExpression5(y=800)
    annotation (Placement(transformation(extent={{-94,-102},{-74,-82}})));
equation
  connect(fixedTemperature.port, heaFloEps.heaPor)
    annotation (Line(points={{-20,-30},{16,-30},{16,-8}}, color={191,0,0}));
  connect(heaFloEps.UA, realExpression1.y) annotation (Line(points={{4,8},{-9,8}},  color={0,0,127}));
  connect(supHea.m_flow_in, ram.y) annotation (Line(points={{-62,8},{-75,8}}, color={0,0,127}));
  connect(supHea.ports[1], heaFloEps.port_a)
    annotation (Line(points={{-40,0},{6,0}},                 color={0,127,255}));
  connect(heaFloEps.port_b, sinHea.ports[1]) annotation (Line(points={{26,0},{44,0},{44,2},{68,2}}, color={0,127,255}));
  connect(supHea1.ports[1], heaOrCoo.port_a) annotation (Line(points={{-42,60},{-10,60}}, color={0,127,255}));
  connect(ram.y, supHea1.m_flow_in) annotation (Line(points={{-75,8},{-70,8},{-70,68},{-64,68}}, color={0,0,127}));
  connect(heaOrCoo.port_b, sinHea.ports[2])
    annotation (Line(points={{10,60},{40,60},{40,-2},{68,-2}}, color={0,127,255}));
  connect(realExpression2.y, simplifiedBuildingODE.TOut)
    annotation (Line(points={{-73,-54},{-32,-54},{-32,-62},{-22,-62}}, color={0,0,127}));
  connect(realExpression3.y, simplifiedBuildingODE.TSet)
    annotation (Line(points={{-73,-66},{-48,-66},{-48,-67.2},{-22,-67.2}}, color={0,0,127}));
  connect(realExpression4.y, simplifiedBuildingODE.Q_flowReq)
    annotation (Line(points={{-73,-78},{-48,-78},{-48,-73},{-22,-73}}, color={0,0,127}));
  connect(realExpression5.y, simplifiedBuildingODE.Q_flowAct)
    annotation (Line(points={{-73,-92},{-48,-92},{-48,-78},{-22,-78}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end Test;
