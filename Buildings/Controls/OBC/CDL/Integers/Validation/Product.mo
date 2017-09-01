within Buildings.Controls.OBC.CDL.Integers.Validation;
model Product "Validation model for the Product block"
extends Modelica.Icons.Example;

  Buildings.Controls.OBC.CDL.Integers.Product product
    "Block that outputs the sum of the two inputs"
    annotation (Placement(transformation(extent={{16,-10},{36,10}})));

  Buildings.Controls.OBC.CDL.Continuous.Truncation truncation1
    "Block that discards the fractional portion of input and provides a whol number output"
    annotation (Placement(transformation(extent={{-28,10},{-8,30}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Ramp ramp1(
    duration=1,
    offset=-3.5,
    height=7.0) "Block that generates ramp signal"
    annotation (Placement(transformation(extent={{-72,10},{-52,30}})));

  Buildings.Controls.OBC.CDL.Continuous.Truncation truncation2
    "Block that discards the fractional portion of input and provides a whol number output"
    annotation (Placement(transformation(extent={{-28,-30},{-8,-10}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Ramp ramp2(
    duration=1,
    offset=-0.5,
    height=7.0) "Block that generates ramp signal"
    annotation (Placement(transformation(extent={{-72,-30},{-52,-10}})));

equation
  connect(ramp1.y, truncation1.u)
    annotation (Line(points={{-51,20},{-30,20},{-30,20}}, color={0,0,127}));
  connect(ramp2.y, truncation2.u) annotation (Line(points={{-51,-20},{-40.5,-20},{-30,
          -20}}, color={0,0,127}));
  connect(truncation1.y, product.u1)
    annotation (Line(points={{-7,20},{4,20},{4,6},{14,6}}, color={255,127,0}));
  connect(truncation2.y, product.u2) annotation (Line(points={{-7,-20},{2,-20},{2,-6},{14,
          -6}}, color={255,127,0}));
  annotation (
  experiment(StopTime=1.0, Tolerance=1e-06),
  __Dymola_Commands(file="modelica://Buildings/Resources/Scripts/Dymola/Controls/OBC/CDL/Integers/Validation/Product.mos"
        "Simulate and plot"),
    Documentation(info="<html>
<p>
Validation test for the block
<a href=\"modelica://Buildings.Controls.OBC.CDL.Integers.Product\">
Buildings.Controls.OBC.CDL.Integers.Product</a>.
</p>
</html>", revisions="<html>
<ul>
<li>
April 1, 2017, by Jianjun Hu:<br/>
First implementation.
</li>
</ul>

</html>"));
end Product;