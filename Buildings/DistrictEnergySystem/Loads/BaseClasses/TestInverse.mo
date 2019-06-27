within Buildings.DistrictEnergySystem.Loads.BaseClasses;
model TestInverse
  EffectivenessDirect testInvBlock annotation (Placement(transformation(extent={{8,-8},{-12,12}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=20*(inv.y1/0.2)^0.8)
    annotation (Placement(transformation(extent={{-98,50},{-78,70}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=273 + 45)
    annotation (Placement(transformation(extent={{-98,34},{-78,54}})));
  Modelica.Blocks.Sources.RealExpression realExpression2(y=4200)
    annotation (Placement(transformation(extent={{-98,18},{-78,38}})));
  Modelica.Blocks.Math.InverseBlockConstraints inv(y1(start=0))
    annotation (Placement(transformation(extent={{-22,24},{18,48}})));
  Modelica.Blocks.Sources.RealExpression realExpression3(y=273 + 20)
    annotation (Placement(transformation(extent={{-98,0},{-78,20}})));
  Modelica.Blocks.Sources.Trapezoid trapezoid(
    amplitude=-400,
    rising=0.5,
    width=0.1,
    falling=0.3,
    period=1) annotation (Placement(transformation(extent={{-98,74},{-78,94}})));
equation
  connect(inv.y2, testInvBlock.m_flow) annotation (Line(points={{15,36},{14,36},{14,6},{10,6}}, color={0,0,127}));
  connect(realExpression.y, testInvBlock.UA)
    annotation (Line(points={{-77,60},{38,60},{38,10},{10,10}}, color={0,0,127}));
  connect(realExpression1.y, testInvBlock.TInl)
    annotation (Line(points={{-77,44},{38,44},{38,2},{10,2}}, color={0,0,127}));
  connect(realExpression2.y, testInvBlock.cpInl)
    annotation (Line(points={{-77,28},{38,28},{38,-2},{10,-2}}, color={0,0,127}));
  connect(realExpression3.y, testInvBlock.TLoad)
    annotation (Line(points={{-77,10},{38,10},{38,-6},{10,-6}}, color={0,0,127}));
  connect(testInvBlock.Q_flow, inv.u2) annotation (Line(points={{-13,2},{-18,2},{-18,36}}, color={0,0,127}));
  connect(trapezoid.y, inv.u1) annotation (Line(points={{-77,84},{-50,84},{-50,36},{-24,36}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(coordinateSystem(preserveAspectRatio=false)));
end TestInverse;
